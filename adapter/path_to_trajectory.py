"""VP DrivingReference -> SI TrajectoryCandidate, sized for the Safety Island.

VisionPilot publishes one compound reference per camera cycle: the lane
polynomial y = a x^2 + b x + c (base_link) plus the planned speed schedule
v(t), carrying the camera source stamp of that cycle. The Safety Island
follower wants an Autoware Trajectory in the same frame as odometry. The
VP-specific envelope preserves the original session/cycle across the adapter.

A reference is transcribed only when it is valid and an ego sample with
the exact same source stamp exists (the bridge stamps both with the CARLA
simulated frame time). Anything else is rejected and nothing is published:
the adapter never authors a stop trajectory, because a synthesized stop
would refresh the selected input and mask the fault the Safety Island must
see. SI owns source freshness and the stop decision.

SI packet budget matches Open AD Kit traj_relay (hardware-validated):
13 points, 25 m extent, 1200 B serialized. ROS imports stay in main() so
the functions below test without a ROS install.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

EXTENT_CAP_M = 25.0
TARGET_POINT_COUNT = 13
SERIALIZED_BYTE_BUDGET = 1300
DEFAULT_TARGET_SPEED_MPS = 3.0
DEFAULT_V_MIN_MPS = 1.0
HORIZON_DT_SEC = 0.05
# Spatial lead (m). Points are read S_LEAD ahead on the transcribed
# schedule instead of at the ego's feet. The SI longitudinal target sits
# decimeters ahead and the bridge map equilibrates at its velocity
# demand, so commanding v(ego) ~= actual speed stalls (proven: 200 s
# crawl at 0.3 m/s with a 1.4 m/s ramp on the table). Reading ahead
# turns the ramp into a rising demand that bootstraps on grades, while
# staying bounded by the schedule end (hn): it can only exceed actual
# speed while the schedule rises, so runaway is structurally impossible.
# A rising-from-zero schedule therefore launches with no special case,
# and a stop schedule still transcribes zeros (SI stop machinery intact).
S_LEAD_M = 1.0
# Near-field arcs (m) always represented so SI's ~0.2 m lookahead target
# lands on the transcribed ramp instead of a coarse downsample chord.
NEAR_ARC_M = (0.25, 0.5, 1.0, 2.0)
DEFAULT_FRAME_ID = "map"

DEFAULT_INPUT_REFERENCE_TOPIC = "/vehicle/driving_reference"
DEFAULT_INPUT_ODOM_TOPIC = "/localization/kinematic_state"
DEFAULT_OUTPUT_TOPIC = "/planning/visionpilot/trajectory_candidate"
# Exact-frame ego buffer depth (~10 s at 20 Hz). A reference whose stamp
# fell off the buffer is rejected like any other unmatched frame.
EGO_BUFFER_DEPTH = 400
# Reject logging period (s): counters are cumulative, the log is not.
REJECT_LOG_PERIOD_SEC = 2.0

CDR_ENCAPSULATION_BYTES = 4
CDR_TIME_BYTES = 8
CDR_UINT32_BYTES = 4
CDR_STRING_ALIGNMENT = 4
CDR_TRAJECTORY_POINT_BYTES = 88
CDR_TRAJECTORY_POINT_ALIGNMENT = 8


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class TrajectoryPoint:
    x: float
    y: float
    yaw: float
    longitudinal_velocity_mps: float
    time_from_start_sec: float
    acceleration_mps2: float = 0.0


def wrap_angle(yaw: float) -> float:
    return math.atan2(math.sin(yaw), math.cos(yaw))


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def sample_quadratic_path(
    a: float,
    b: float,
    c: float,
    x_max_m: float,
    spacing_m: float = 1.0,
) -> list[Pose2D]:
    """Sample VP's RANSAC polynomial y = a x² + b x + c in base_link.

    x forward [m], y left [m]. Tangent yaw = atan(2ax + b).
    """
    if x_max_m <= 0.0 or spacing_m <= 0.0:
        return []
    points: list[Pose2D] = []
    x = 0.0
    while x <= x_max_m + 1e-9:
        y = a * x * x + b * x + c
        yaw = math.atan(2.0 * a * x + b)
        points.append(Pose2D(x, y, yaw))
        x += spacing_m
    return points


def transform_to_odom(point: Pose2D, ego: Pose2D) -> Pose2D:
    """base_link (x forward, y left) → odom/map using ego pose."""
    cos_y = math.cos(ego.yaw)
    sin_y = math.sin(ego.yaw)
    return Pose2D(
        x=ego.x + point.x * cos_y - point.y * sin_y,
        y=ego.y + point.x * sin_y + point.y * cos_y,
        yaw=wrap_angle(ego.yaw + point.yaw),
    )


def target_speed_mps(
    target_mps: float = DEFAULT_TARGET_SPEED_MPS,
    v_min_mps: float = DEFAULT_V_MIN_MPS,
) -> float:
    return max(v_min_mps, target_mps)


def _align_up(offset: int, alignment: int) -> int:
    return (offset + alignment - 1) // alignment * alignment


def serialized_size_bytes(frame_id_length: int, point_count: int) -> int:
    offset = CDR_TIME_BYTES
    offset += CDR_UINT32_BYTES
    offset += _align_up(frame_id_length + 1, CDR_STRING_ALIGNMENT)
    offset += CDR_UINT32_BYTES
    if point_count > 0:
        offset = _align_up(offset, CDR_TRAJECTORY_POINT_ALIGNMENT)
        offset += CDR_TRAJECTORY_POINT_BYTES * point_count
    return CDR_ENCAPSULATION_BYTES + offset


def candidate_serialized_size_bytes(frame_id_length: int, point_count: int) -> int:
    """CDR size of TrajectoryCandidate (trajectory + uint32 + uint64)."""
    offset = serialized_size_bytes(frame_id_length, point_count) - CDR_ENCAPSULATION_BYTES
    offset = _align_up(offset, 4) + 4  # source_session
    offset = _align_up(offset, 8) + 8  # source_cycle
    return CDR_ENCAPSULATION_BYTES + offset


def max_points_within(frame_id_length: int, byte_budget: int) -> int:
    count = 0
    while candidate_serialized_size_bytes(frame_id_length, count + 1) <= byte_budget:
        count += 1
    return count


def cumulative_arc_lengths(positions: list[tuple[float, float]]) -> list[float]:
    lengths = [0.0] * len(positions)
    for i in range(1, len(positions)):
        px, py = positions[i - 1]
        cx, cy = positions[i]
        lengths[i] = lengths[i - 1] + math.hypot(cx - px, cy - py)
    return lengths


def _nearest_index(arc_lengths: list[float], target: float) -> int:
    lo, hi = 0, len(arc_lengths)
    while lo < hi:
        mid = (lo + hi) // 2
        if arc_lengths[mid] < target:
            lo = mid + 1
        else:
            hi = mid
    position = lo
    if position == 0:
        return 0
    if position >= len(arc_lengths):
        return len(arc_lengths) - 1
    after = arc_lengths[position] - target
    before = target - arc_lengths[position - 1]
    return position if after < before else position - 1


def select_indices(
    arc_lengths: list[float], extent_cap_m: float, max_points: int
) -> list[int]:
    count = len(arc_lengths)
    if count == 0:
        return []
    if count == 1 or max_points <= 1:
        return [0]
    extent = min(extent_cap_m, arc_lengths[-1])
    if extent <= 0.0:
        return [0]
    wanted = min(max_points, count)
    if wanted < 2:
        return [0]
    indices: list[int] = []
    for step in range(wanted):
        target = extent * step / (wanted - 1)
        index = _nearest_index(arc_lengths, target)
        if not indices or index > indices[-1]:
            indices.append(index)
    return indices


def downsample_indices(
    positions: list[tuple[float, float]],
    frame_id_length: int,
    extent_cap_m: float = EXTENT_CAP_M,
    target_point_count: int = TARGET_POINT_COUNT,
    byte_budget: int = SERIALIZED_BYTE_BUDGET,
) -> list[int]:
    budget = min(
        target_point_count, max_points_within(frame_id_length, byte_budget)
    )
    if budget <= 0:
        return []
    return select_indices(
        cumulative_arc_lengths(positions), extent_cap_m, budget
    )


def stamp_key(sec: int, nanosec: int) -> tuple[int, int]:
    """Exact simulator-frame identity: the CARLA sim time on a stamp."""
    return (int(sec), int(nanosec))


def reference_reason(
    *,
    valid: bool,
    path_valid: bool,
    has_source_stamp: bool,
    horizon_len: int,
    horizon_dt_s: float,
    x_max_m: float,
) -> str:
    """Why a VP reference must be rejected, or "" when usable.

    Pure policy. Rejection is silence for the follower: the adapter never
    authors a stop, because a synthesized stop would refresh the selected
    input and mask the very fault SI must detect and answer.
    """
    if not valid:
        return "invalid-reference"
    if not has_source_stamp:
        return "no-source-stamp"
    if not path_valid or x_max_m <= 0.0:
        return "no-path"
    if horizon_len < 2 or horizon_dt_s <= 0.0:
        return "no-horizon"
    return ""


def cycle_reason(session: int, cycle: int, last_session: int, last_cycle: int) -> str:
    """Why this cycle must not be accepted after an earlier one.

    Duplicates and regressions within one VP session are stale data that
    must not become fresh again by republication; a new session (VP
    restart) is accepted and resets the comparison.
    """
    if session == last_session and cycle <= last_cycle:
        return "cycle-not-increasing"
    return ""


def sample_horizon(horizon: list[float], t_sec: float, dt_sec: float = HORIZON_DT_SEC) -> float:
    if not horizon or dt_sec <= 0.0:
        return 0.0
    if t_sec <= 0.0:
        return max(0.0, horizon[0])
    idx = t_sec / dt_sec
    i = int(idx)
    if i >= len(horizon) - 1:
        return max(0.0, horizon[-1])
    frac = idx - i
    return max(0.0, horizon[i] * (1.0 - frac) + horizon[i + 1] * frac)


RESAMPLE_STEP_M = 0.25


def resample_path(points: list[Pose2D], step_m: float = RESAMPLE_STEP_M) -> list[Pose2D]:
    """Densify a polyline so sub-meter schedule detail survives indexing.

    The VP lane path arrives at 1 m spacing; linear resampling between
    those samples is faithful (smooth polynomial) and only affects
    internal resolution — the emitted point budget is unchanged.
    """
    if len(points) < 2 or step_m <= 0.0:
        return list(points)
    out = [points[0]]
    for a, b in zip(points, points[1:]):
        seg = math.hypot(b.x - a.x, b.y - a.y)
        if seg <= 1e-9:
            continue
        dyaw = wrap_angle(b.yaw - a.yaw)
        n = max(1, int(math.ceil(seg / step_m)))
        for k in range(1, n + 1):
            frac = k / n
            out.append(
                Pose2D(
                    a.x + (b.x - a.x) * frac,
                    a.y + (b.y - a.y) * frac,
                    wrap_angle(a.yaw + dyaw * frac),
                )
            )
    return out


def horizon_arc_lengths(
    horizon: list[float], dt_sec: float = HORIZON_DT_SEC
) -> list[float]:
    """Integrate a temporal speed schedule v(t) into arc positions s(t).

    Trapezoid rule. Exact zeros in the horizon stay exact zeros in s(t),
    so a VP stop schedule transcribes to zero-velocity arc points that
    the SI follower reads as a stop (its stop machinery keys off those).
    """
    s = [0.0]
    for i in range(1, len(horizon)):
        v0 = max(0.0, horizon[i - 1])
        v1 = max(0.0, horizon[i])
        s.append(s[-1] + 0.5 * (v0 + v1) * dt_sec)
    return s


def sample_spatial(
    s_knots: list[float],
    v_knots: list[float],
    s: float,
    hold: float,
    dt_sec: float = HORIZON_DT_SEC,
) -> tuple[float, float]:
    """Speed and schedule-time at arc s.

    Inside the horizon extent, linear interpolation of the transcribed
    schedule. Beyond it, hold the last horizon speed (there is no VP
    information there; the SI longitudinal target lives <1 m ahead, so
    the hold only feeds the far field).
    """
    if not s_knots:
        return max(0.0, hold), 0.0
    if s <= 0.0:
        return max(0.0, v_knots[0]), 0.0
    if s >= s_knots[-1]:
        extra = s - s_knots[-1]
        t = (len(s_knots) - 1) * dt_sec + extra / max(hold, 0.3)
        return max(0.0, hold), t
    lo, hi = 0, len(s_knots) - 1
    while lo + 1 < hi:
        mid = (lo + hi) // 2
        if s_knots[mid] < s:
            lo = mid
        else:
            hi = mid
    seg = s_knots[hi] - s_knots[lo]
    frac = 0.0 if seg <= 1e-9 else (s - s_knots[lo]) / seg
    v = v_knots[lo] * (1.0 - frac) + v_knots[hi] * frac
    return max(0.0, v), (lo + frac) * dt_sec


def profile_accelerations(
    speeds: list[float], times: list[float], fallback: float = 0.0
) -> list[float]:
    """Per-point dv/dt so the accel field agrees with the speed profile.

    A constant accel copied onto a flat speed profile makes the SI
    feedforward fight its velocity loop; differentiating keeps them
    consistent (0 on holds and stop tails, slope on ramps).
    """
    n = len(speeds)
    if n == 0:
        return []
    if n == 1:
        return [fallback]
    out = [0.0] * n
    for i in range(n):
        if i == 0:
            dt = times[1] - times[0]
            out[i] = (speeds[1] - speeds[0]) / dt if dt > 1e-6 else fallback
        elif i == n - 1:
            dt = times[-1] - times[-2]
            out[i] = (speeds[-1] - speeds[-2]) / dt if dt > 1e-6 else 0.0
        else:
            dt = times[i + 1] - times[i - 1]
            out[i] = (speeds[i + 1] - speeds[i - 1]) / dt if dt > 1e-6 else 0.0
    return out


def convert(
    path_points: list[Pose2D],
    ego: Pose2D,
    *,
    target_velocity_mps: float | None = None,
    speed_horizon: list[float] | None = None,
    vp_accel: float | None = None,
    horizon_dt_sec: float = HORIZON_DT_SEC,
    v_min_mps: float = DEFAULT_V_MIN_MPS,
    frame_id: str = DEFAULT_FRAME_ID,
    extent_cap_m: float = EXTENT_CAP_M,
    target_point_count: int = TARGET_POINT_COUNT,
    byte_budget: int = SERIALIZED_BYTE_BUDGET,
) -> list[TrajectoryPoint] | None:
    if not path_points:
        return None
    if not speed_horizon and target_velocity_mps is None:
        return None

    world = [transform_to_odom(p, ego) for p in path_points]
    world = resample_path(world)
    positions = [(p.x, p.y) for p in world]
    indices = downsample_indices(
        positions,
        len(frame_id),
        extent_cap_m=extent_cap_m,
        target_point_count=target_point_count,
        byte_budget=byte_budget,
    )
    if not indices:
        return None
    if indices[0] != 0:
        indices = [0] + [i for i in indices if i != 0]
        budget = min(
            target_point_count, max_points_within(len(frame_id), byte_budget)
        )
        indices = indices[:budget]
    if candidate_serialized_size_bytes(len(frame_id), len(indices)) > byte_budget:
        return None

    if speed_horizon:
        # Force near-field samples: the SI longitudinal target sits a few
        # decimeters ahead, and a coarse 25 m downsample would chord over
        # the sub-meter ramp/stop detail transcribed from the 1 s horizon.
        full_lengths = cumulative_arc_lengths(positions)
        extent = min(extent_cap_m, full_lengths[-1]) if full_lengths else 0.0
        extra = set()
        for want in NEAR_ARC_M:
            if 0.0 < want < extent:
                extra.add(_nearest_index(full_lengths, want))
        if extra:
            budget = min(
                target_point_count, max_points_within(len(frame_id), byte_budget)
            )
            # Merge, then shed far points first if over budget: the near
            # field carries the stop/ramp detail, the far field is a hold.
            indices = sorted(set(indices) | extra)
            while len(indices) > max(budget, 1):
                indices = indices[: len(indices) - 1]

    selected = [world[i] for i in indices]
    if math.hypot(selected[0].x - ego.x, selected[0].y - ego.y) > 0.5:
        selected = [ego] + selected
        budget = min(
            target_point_count, max_points_within(len(frame_id), byte_budget)
        )
        selected = selected[:budget]
    lengths = cumulative_arc_lengths([(p.x, p.y) for p in selected])
    out: list[TrajectoryPoint] = []
    cruise = (
        None
        if speed_horizon
        else target_speed_mps(target_velocity_mps or 0.0, v_min_mps)
    )
    speeds: list[float] = []
    times: list[float] = []
    accels: list[float] = []
    if speed_horizon:
        # Faithful transcription: VP's 1 s speed schedule v(t) becomes a
        # spatial profile v(s) via s(t) = ∫v dt. Ramp, hold, and stop-tail
        # (exact zeros) all survive, so the SI stop machinery sees stops
        # as trajectory properties instead of momentary velocity values.
        h = [max(0.0, v) for v in speed_horizon]
        hn = h[-1]
        s_knots = horizon_arc_lengths(h, horizon_dt_sec)
        for s in lengths:
            v, t = sample_spatial(s_knots, h, s + S_LEAD_M, hn, horizon_dt_sec)
            speeds.append(v)
            times.append(t)
        t_h = max(horizon_dt_sec * (len(h) - 1), horizon_dt_sec)
        if vp_accel is None and t_h > 0.0:
            fallback = (hn - h[0]) / t_h
        elif vp_accel is not None:
            fallback = vp_accel
        else:
            fallback = 0.0
        accels = profile_accelerations(speeds, times, fallback)
    else:
        for s in lengths:
            speed = cruise if cruise is not None else 0.0
            speeds.append(speed)
            times.append(0.0 if speed <= 0.0 else s / speed)
            accels.append(0.0)
    for i, p in enumerate(selected):
        out.append(
            TrajectoryPoint(
                x=p.x,
                y=p.y,
                yaw=p.yaw,
                longitudinal_velocity_mps=speeds[i],
                time_from_start_sec=times[i],
                acceleration_mps2=accels[i],
            )
        )
    return out


def main(args=None) -> None:
    import rclpy
    from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint as TrajMsg
    from builtin_interfaces.msg import Duration, Time
    from geometry_msgs.msg import Pose
    from nav_msgs.msg import Odometry
    from safety_island_msgs.msg import TrajectoryCandidate
    from visionpilot_msgs.msg import DrivingReference
    import time as time_mod
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import (
        QoSDurabilityPolicy,
        QoSHistoryPolicy,
        QoSProfile,
        QoSReliabilityPolicy,
    )

    rclpy.init(args=args)
    node = rclpy.create_node("path_to_trajectory")

    input_reference = node.declare_parameter(
        "input_reference_topic", DEFAULT_INPUT_REFERENCE_TOPIC
    ).value
    input_odom = node.declare_parameter(
        "input_odom_topic", DEFAULT_INPUT_ODOM_TOPIC
    ).value
    output_topic = node.declare_parameter("output_topic", DEFAULT_OUTPUT_TOPIC).value
    v_min = node.declare_parameter("v_min_mps", DEFAULT_V_MIN_MPS).value
    out_frame = node.declare_parameter("frame_id", DEFAULT_FRAME_ID).value
    extent_cap = node.declare_parameter("extent_cap_m", EXTENT_CAP_M).value
    point_count = node.declare_parameter(
        "target_point_count", TARGET_POINT_COUNT
    ).value
    byte_budget = node.declare_parameter(
        "byte_budget", SERIALIZED_BYTE_BUDGET
    ).value

    qos = QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
    )
    publisher = node.create_publisher(TrajectoryCandidate, output_topic, qos)
    # Ego samples keyed by the CARLA sim-time stamp on their header: a
    # reference is transcribed only when its source stamp finds the
    # identical key, i.e. the path and the pose are from one world frame.
    ego_by_key: dict[tuple[int, int], tuple[Pose2D, float]] = {}
    odom_at: dict[tuple[int, int], float] = {}
    ego_order: list[tuple[int, int]] = []
    last_session = [-1]
    last_cycle = [-1]
    published_count = [0]
    rejects: dict[str, int] = {}
    last_reject_log_at = [0.0]

    def publish_points(
        points: list[TrajectoryPoint], stamp, source_session: int, source_cycle: int
    ) -> None:
        out = Trajectory()
        out.header.stamp = stamp
        out.header.frame_id = out_frame
        for p in points:
            tp = TrajMsg()
            tp.pose = Pose()
            tp.pose.position.x = p.x
            tp.pose.position.y = p.y
            qx, qy, qz, qw = quaternion_from_yaw(p.yaw)
            tp.pose.orientation.x = qx
            tp.pose.orientation.y = qy
            tp.pose.orientation.z = qz
            tp.pose.orientation.w = qw
            tp.longitudinal_velocity_mps = p.longitudinal_velocity_mps
            tp.acceleration_mps2 = p.acceleration_mps2
            sec = int(p.time_from_start_sec)
            nsec = int(round((p.time_from_start_sec - sec) * 1e9))
            tp.time_from_start = Duration(sec=sec, nanosec=nsec)
            out.points.append(tp)
        candidate = TrajectoryCandidate()
        candidate.trajectory = out
        candidate.source_session = source_session
        candidate.source_cycle = source_cycle
        publisher.publish(candidate)

    def reject(reason: str) -> None:
        # Rejection is silence: the adapter does not author a stop, so a
        # stale or mismatched reference cannot refresh SI's input. The
        # counters make the reason visible without log spam.
        rejects[reason] = rejects.get(reason, 0) + 1
        now = time_mod.monotonic()
        if now - last_reject_log_at[0] > REJECT_LOG_PERIOD_SEC:
            last_reject_log_at[0] = now
            totals = " ".join(
                "%s=%d" % (k, v) for k, v in sorted(rejects.items())
            )
            node.get_logger().warn("reject %s (totals %s)" % (reason, totals))

    def on_odom(msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        key = stamp_key(msg.header.stamp.sec, msg.header.stamp.nanosec)
        if key not in ego_by_key:
            ego_order.append(key)
        ego_by_key[key] = (
            Pose2D(
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                yaw_from_quaternion(q.x, q.y, q.z, q.w),
            ),
            float(msg.twist.twist.linear.x),
        )
        odom_at[key] = time_mod.monotonic()
        while len(ego_order) > EGO_BUFFER_DEPTH:
            drop = ego_order.pop(0)
            ego_by_key.pop(drop, None)
            odom_at.pop(drop, None)

    def on_reference(msg: DrivingReference) -> None:
        reason = reference_reason(
            valid=msg.valid,
            path_valid=msg.path_valid,
            has_source_stamp=msg.has_source_stamp,
            horizon_len=len(msg.speed_horizon_mps),
            horizon_dt_s=msg.horizon_dt_s,
            x_max_m=msg.path_x_max_m,
        )
        if reason:
            reject(reason)
            return
        key = stamp_key(msg.source_stamp.sec, msg.source_stamp.nanosec)
        sample = ego_by_key.get(key)
        if sample is None:
            reject("no-same-frame-ego")
            return
        reason = cycle_reason(
            msg.session, msg.cycle, last_session[0], last_cycle[0]
        )
        if reason:
            reject(reason)
            return
        ego, odv = sample
        path_points = sample_quadratic_path(
            msg.path_a, msg.path_b, msg.path_c, msg.path_x_max_m
        )
        if not path_points:
            reject("empty-path")
            return
        horizon = [max(0.0, float(v)) for v in msg.speed_horizon_mps]
        points = convert(
            path_points,
            ego,
            speed_horizon=horizon,
            horizon_dt_sec=msg.horizon_dt_s,
            v_min_mps=v_min,
            frame_id=out_frame,
            extent_cap_m=extent_cap,
            target_point_count=point_count,
            byte_budget=byte_budget,
        )
        if points is None:
            reject("bad-shape")
            return
        publish_points(
            points,
            Time(
                sec=int(msg.source_stamp.sec),
                nanosec=int(msg.source_stamp.nanosec),
            ),
            msg.session,
            msg.cycle,
        )
        last_session[0] = msg.session
        last_cycle[0] = msg.cycle
        published_count[0] += 1
        now = time_mod.monotonic()
        s_h = 0.0
        for i in range(1, len(horizon)):
            s_h += 0.5 * (horizon[i - 1] + horizon[i]) * msg.horizon_dt_s
        stop0 = next((i for i, v in enumerate(horizon) if v <= 1e-6), -1)
        size = candidate_serialized_size_bytes(len(out_frame), len(points))
        node.get_logger().info(
            f"xfer #{published_count[0]} sess={msg.session} cyc={msg.cycle} "
            f"src={msg.source_stamp.sec}.{msg.source_stamp.nanosec:09d} "
            f"h0={horizon[0]:.2f} hn={horizon[-1]:.2f} n={len(horizon)} "
            f"s_h={s_h:.1f} stop0={stop0} "
            f"v0={points[0].longitudinal_velocity_mps:.2f} "
            f"vn={points[-1].longitudinal_velocity_mps:.2f} "
            f"traj_a0={points[0].acceleration_mps2:.2f} odv={odv:.2f} "
            f"o_age={(now - odom_at.get(key, now)) * 1000.0:.0f}ms "
            f"pts={len(points)} B={size} rej={sum(rejects.values())}"
        )

    node.create_subscription(Odometry, input_odom, on_odom, qos)
    node.create_subscription(DrivingReference, input_reference, on_reference, qos)
    # No watchdog and no stop publication: silence means "no usable
    # reference", and the stop decision (and its timing) belongs to SI.
    node.get_logger().info(
        f"{input_reference} + {input_odom} -> {output_topic} "
        f"(same-frame ego match required, rejects are silent; "
        f"≤{point_count} pts, {extent_cap} m, {byte_budget} B)"
    )
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
