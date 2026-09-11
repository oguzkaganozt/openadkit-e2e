"""Path (base_link) → Autoware Trajectory, sized for the Safety Island.

VisionPilot publishes a lane-center Path in base_link. The Safety Island
follower wants autoware_planning_msgs/Trajectory in the same frame as
odometry, small enough to cross DDS without fragmentation.

SI packet budget matches Open AD Kit traj_relay (hardware-validated):
13 points, 25 m extent, 1200 B serialized. ROS imports stay in main() so
the functions below test without a ROS install.
"""

from __future__ import annotations

import math
import os
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

CDR_ENCAPSULATION_BYTES = 4
CDR_TIME_BYTES = 8
CDR_UINT32_BYTES = 4
CDR_STRING_ALIGNMENT = 4
CDR_TRAJECTORY_POINT_BYTES = 88
CDR_TRAJECTORY_POINT_ALIGNMENT = 8

DEFAULT_INPUT_PATH_TOPIC = "/vehicle/lane_path"
DEFAULT_INPUT_ODOM_TOPIC = "/localization/kinematic_state"
DEFAULT_INPUT_HORIZON_TOPIC = "/vehicle/speed_horizon"
DEFAULT_INPUT_ACCEL_TOPIC = "/vehicle/throttle_cmd"
DEFAULT_OUTPUT_TOPIC = "/planning/scenario_planning/trajectory"
# Ingress staleness bound (ms). Healthy runs show horizon age 50-270 ms
# (xfer h_age) and odom at 20 Hz, so 1000 ms is >3x the worst healthy
# sample — not a copied 0.5 s. Anything older means VP (or the bridge
# odom feed) is dead, and the answer is an explicit stop trajectory,
# never silence: SI latches its last trajectory (has_trajectory_ never
# clears), so silence would drive stale forever.
STALE_INPUT_MS = 1000.0
# Watchdog period bound (s). If no fresh Path arrives at all (VP fully
# dead), the on_path callback never fires, so a timer publishes the stop.
STOP_WATCHDOG_SEC = 1.0


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


def max_points_within(frame_id_length: int, byte_budget: int) -> int:
    count = 0
    while serialized_size_bytes(frame_id_length, count + 1) <= byte_budget:
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


def stop_trajectory(ego: Pose2D) -> list[TrajectoryPoint]:
    p1 = transform_to_odom(Pose2D(0.25, 0.0, 0.0), ego)
    p2 = transform_to_odom(Pose2D(0.50, 0.0, 0.0), ego)
    return [
        TrajectoryPoint(ego.x, ego.y, ego.yaw, 0.0, 0.0, acceleration_mps2=-1.0),
        TrajectoryPoint(p1.x, p1.y, p1.yaw, 0.0, 0.25, acceleration_mps2=-1.0),
        TrajectoryPoint(p2.x, p2.y, p2.yaw, 0.0, 0.50, acceleration_mps2=-1.0),
    ]


def ingress_reason(
    now_ms: float,
    horizon_at_ms: float,
    odom_at_ms: float,
    stale_ms: float = STALE_INPUT_MS,
) -> str:
    """Why the latest input must not be trusted, or "" when fresh.

    Pure policy: no sequence exists on this wire (Float32MultiArray has
    no seq field), so arrival age is the only staleness signal. A stale
    or missing horizon/odom maps to an explicit stop downstream.
    """
    if horizon_at_ms <= 0.0:
        return "no-horizon-yet"
    if now_ms - horizon_at_ms > stale_ms:
        return "stale-horizon"
    if odom_at_ms <= 0.0:
        return "no-odom-yet"
    if now_ms - odom_at_ms > stale_ms:
        return "stale-odom"
    return ""


def watchdog_due(
    now_s: float,
    last_path_at_s: float,
    period_s: float = STOP_WATCHDOG_SEC,
) -> bool:
    """True when no fresh Path arrived within the watchdog period."""
    if last_path_at_s <= 0.0:
        return True
    return now_s - last_path_at_s > period_s


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
        return stop_trajectory(ego)
    if not speed_horizon and target_velocity_mps is None:
        return stop_trajectory(ego)

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
    if serialized_size_bytes(len(frame_id), len(indices)) > byte_budget:
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
    from builtin_interfaces.msg import Duration
    from geometry_msgs.msg import Pose
    from nav_msgs.msg import Odometry, Path
    from std_msgs.msg import Bool, Float32MultiArray, Float64
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

    input_path = node.declare_parameter(
        "input_path_topic", DEFAULT_INPUT_PATH_TOPIC
    ).value
    input_odom = node.declare_parameter(
        "input_odom_topic", DEFAULT_INPUT_ODOM_TOPIC
    ).value
    input_horizon = node.declare_parameter(
        "input_horizon_topic", DEFAULT_INPUT_HORIZON_TOPIC
    ).value
    input_accel = node.declare_parameter(
        "input_accel_topic", DEFAULT_INPUT_ACCEL_TOPIC
    ).value
    inhibit_topic = node.declare_parameter(
        "inhibit_topic", "/guard/inhibit_trajectory"
    ).value
    inhibited: list[bool] = [False]

    def on_inhibit(msg: Bool) -> None:
        inhibited[0] = bool(msg.data)
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
    # A/B baseline knob: >0 ignores the VP horizon and drives the VP path
    # shape at a constant speed through SI. 0.0 (default) = VP horizon.
    cruise_override = float(
        node.declare_parameter(
            "cruise_override_mps",
            float(os.environ.get("CRUISE_OVERRIDE_MPS", "0.0")),
        ).value
    )

    qos = QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
    )
    publisher = node.create_publisher(Trajectory, output_topic, qos)
    latest_ego: list[Pose2D | None] = [None]
    latest_odom_v: list[float] = [0.0]
    odom_at: list[float] = [0.0]
    latest_horizon: list[list[float] | None] = [None]
    latest_accel: list[float | None] = [None]
    horizon_at: list[float] = [0.0]
    accel_at: list[float] = [0.0]
    published_count = [0]
    path_at: list[float] = [0.0]
    stops_published = [0]
    last_stop_log_at = [0.0]

    def publish_points(points: list[TrajectoryPoint], stamp) -> None:
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
        publisher.publish(out)

    def publish_stop(ego: Pose2D, reason: str) -> None:
        # The defined answer to stale/invalid input: an explicit stop
        # trajectory, never silence. SI latches its last trajectory, so
        # silence would drive stale forever.
        publish_points(stop_trajectory(ego), node.get_clock().now().to_msg())
        stops_published[0] += 1
        now = time_mod.monotonic()
        if now - last_stop_log_at[0] > 2.0:
            last_stop_log_at[0] = now
            node.get_logger().warn(
                "stop #%d reason=%s" % (stops_published[0], reason)
            )

    def on_odom(msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        latest_ego[0] = Pose2D(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw_from_quaternion(q.x, q.y, q.z, q.w),
        )
        latest_odom_v[0] = float(msg.twist.twist.linear.x)
        odom_at[0] = time_mod.monotonic()

    def on_horizon(msg: Float32MultiArray) -> None:
        latest_horizon[0] = [float(v) for v in msg.data]
        horizon_at[0] = time_mod.monotonic()

    def on_accel(msg: Float64) -> None:
        latest_accel[0] = float(msg.data)
        accel_at[0] = time_mod.monotonic()

    def on_path(msg: Path) -> None:
        ego = latest_ego[0]
        if ego is None:
            return
        now = time_mod.monotonic()
        path_at[0] = now
        if inhibited[0]:
            publish_stop(ego, "inhibited")
            return
        now_ms = now * 1000.0
        reason = ingress_reason(
            now_ms, horizon_at[0] * 1000.0, odom_at[0] * 1000.0
        )
        if reason and cruise_override <= 0.0:
            publish_stop(ego, reason)
            return
        path_points = []
        for ps in msg.poses:
            q = ps.pose.orientation
            path_points.append(
                Pose2D(
                    ps.pose.position.x,
                    ps.pose.position.y,
                    yaw_from_quaternion(q.x, q.y, q.z, q.w),
                )
            )
        if not path_points:
            publish_stop(ego, "empty-path")
            return
        hz = None if cruise_override > 0.0 else latest_horizon[0]
        if hz is None and cruise_override <= 0.0:
            publish_stop(ego, "no-horizon-yet")
            return
        points = convert(
            path_points,
            ego,
            target_velocity_mps=(
                cruise_override if cruise_override > 0.0 else None
            ),
            speed_horizon=hz,
            vp_accel=latest_accel[0],
            v_min_mps=v_min,
            frame_id=out_frame,
            extent_cap_m=extent_cap,
            target_point_count=point_count,
            byte_budget=byte_budget,
        )
        if points is None:
            publish_stop(ego, "bad-shape")
            return
        publish_points(points, msg.header.stamp)
        published_count[0] += 1
        vp_a = latest_accel[0]
        a_age = (now - accel_at[0]) * 1000.0 if accel_at[0] else -1.0
        h_age = (now - horizon_at[0]) * 1000.0 if horizon_at[0] else -1.0
        v0 = points[0].longitudinal_velocity_mps
        vn = points[-1].longitudinal_velocity_mps
        a0 = points[0].acceleration_mps2
        odv = latest_odom_v[0]
        o_age = (now - odom_at[0]) * 1000.0 if odom_at[0] else -1.0
        if cruise_override > 0.0:
            hinfo = "cruise-override %.1f" % cruise_override
        elif hz:
            hc = [max(0.0, v) for v in hz]
            s_h = 0.0
            for i in range(1, len(hc)):
                s_h += 0.5 * (hc[i - 1] + hc[i]) * HORIZON_DT_SEC
            stop0 = next(
                (i for i, v in enumerate(hc) if v <= 1e-6), -1
            )
            hinfo = (
                f"h0={hc[0]:.2f} hn={hc[-1]:.2f} n={len(hc)} "
                f"s_h={s_h:.1f} stop0={stop0}"
            )
        else:
            hinfo = "no-horizon"
        node.get_logger().info(
            f"xfer #{published_count[0]} vp_a={vp_a if vp_a is not None else 'NA'} "
            f"a_age={a_age:.0f}ms h_age={h_age:.0f}ms {hinfo} "
            f"v0={v0:.2f} vn={vn:.2f} traj_a0={a0:.2f} "
            f"odv={odv:.2f} o_age={o_age:.0f}ms"
        )

    def on_watchdog() -> None:
        # VP fully dead: no Path arrives, so on_path never fires. Publish
        # the stop on a timer instead of going silent (SI would drive the
        # last trajectory forever). Also clears the zombie horizon so
        # motion cannot resume without a fresh Path alongside fresh input.
        now = time_mod.monotonic()
        ego = latest_ego[0]
        if ego is None:
            return
        if watchdog_due(now, path_at[0]):
            latest_horizon[0] = None
            horizon_at[0] = 0.0
            publish_stop(ego, "watchdog-no-path")

    node.create_subscription(Odometry, input_odom, on_odom, qos)
    node.create_subscription(Path, input_path, on_path, qos)
    node.create_subscription(Float32MultiArray, input_horizon, on_horizon, qos)
    node.create_subscription(Float64, input_accel, on_accel, qos)
    node.create_subscription(Bool, inhibit_topic, on_inhibit, qos)
    node.create_timer(0.2, on_watchdog)
    node.get_logger().info(
        f"{input_path} + {input_odom} + {input_horizon} -> {output_topic} "
        f"(≤{point_count} pts, {extent_cap} m, {byte_budget} B, "
        f"stale>{STALE_INPUT_MS:.0f}ms=stop, watchdog {STOP_WATCHDOG_SEC:.1f}s"
        + (
            ", cruise-override %.1f m/s" % cruise_override
            if cruise_override > 0.0
            else ""
        )
        + ")"
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
