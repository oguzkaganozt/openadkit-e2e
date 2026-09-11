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
from dataclasses import dataclass

EXTENT_CAP_M = 25.0
TARGET_POINT_COUNT = 13
SERIALIZED_BYTE_BUDGET = 1300
DEFAULT_TARGET_SPEED_MPS = 3.0
DEFAULT_V_MIN_MPS = 1.0
HORIZON_DT_SEC = 0.05
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
DEFAULT_OUTPUT_TOPIC = "/planning/scenario_planning/trajectory"


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


def convert(
    path_points: list[Pose2D],
    ego: Pose2D,
    *,
    target_velocity_mps: float | None = None,
    speed_horizon: list[float] | None = None,
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

    selected = [world[i] for i in indices]
    if math.hypot(selected[0].x - ego.x, selected[0].y - ego.y) > 0.5:
        selected = [ego] + selected
        budget = min(
            target_point_count, max_points_within(len(frame_id), byte_budget)
        )
        selected = selected[:budget]
    lengths = cumulative_arc_lengths([(p.x, p.y) for p in selected])
    out: list[TrajectoryPoint] = []
    t = 0.0
    cruise = (
        None
        if speed_horizon
        else target_speed_mps(target_velocity_mps or 0.0, v_min_mps)
    )
    for i, p in enumerate(selected):
        if speed_horizon:
            speed = sample_horizon(speed_horizon, t, horizon_dt_sec)
            # Horizon[0] is current ego speed. From rest the planned
            # acceleration lives in later samples — use the next tick so
            # SI does not treat a launching path as a stop.
            if speed < 0.2 and max(speed_horizon) > 1.0:
                # From rest, command VP's 0.5 s intent so the follower
                # launches instead of tracking a 0 m/s first point.
                speed = sample_horizon(speed_horizon, 0.5, horizon_dt_sec)
        else:
            speed = cruise if cruise is not None else 0.0
        ds_next = (lengths[i + 1] - lengths[i]) if i + 1 < len(lengths) else 0.0
        accel = 0.0
        if speed_horizon and ds_next > 0.0:
            v_next = sample_horizon(
                speed_horizon, t + ds_next / max(speed, 0.1), horizon_dt_sec
            )
            dt_seg = ds_next / max(speed, 0.1)
            accel = (v_next - speed) / dt_seg
        out.append(
            TrajectoryPoint(
                x=p.x,
                y=p.y,
                yaw=p.yaw,
                longitudinal_velocity_mps=speed,
                time_from_start_sec=t,
                acceleration_mps2=accel,
            )
        )
        if i + 1 < len(lengths):
            # Advance along the time horizon even from rest so later points
            # pick up planned acceleration (horizon[0] is current ego speed).
            t += ds_next / max(speed, 0.5)
    return out


def main(args=None) -> None:
    import rclpy
    from autoware_planning_msgs.msg import Trajectory, TrajectoryPoint as TrajMsg
    from builtin_interfaces.msg import Duration
    from geometry_msgs.msg import Pose
    from nav_msgs.msg import Odometry, Path
    from std_msgs.msg import Float32MultiArray
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
    publisher = node.create_publisher(Trajectory, output_topic, qos)
    latest_ego: list[Pose2D | None] = [None]
    latest_horizon: list[list[float] | None] = [None]
    published_count = [0]

    def on_odom(msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        latest_ego[0] = Pose2D(
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            yaw_from_quaternion(q.x, q.y, q.z, q.w),
        )

    def on_horizon(msg: Float32MultiArray) -> None:
        latest_horizon[0] = [float(v) for v in msg.data]
    def on_path(msg: Path) -> None:
        ego = latest_ego[0]
        if ego is None:
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
        points = convert(
            path_points,
            ego,
            speed_horizon=latest_horizon[0],
            v_min_mps=v_min,
            frame_id=out_frame,
            extent_cap_m=extent_cap,
            target_point_count=point_count,
            byte_budget=byte_budget,
        )
        if points is None:
            return
        out = Trajectory()
        out.header.stamp = msg.header.stamp
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
        published_count[0] += 1
        if published_count[0] == 1 or published_count[0] % 100 == 0:
            first = out.points[0].pose.position
            last = out.points[-1].pose.position
            v0 = out.points[0].longitudinal_velocity_mps
            vn = out.points[-1].longitudinal_velocity_mps
            hz = latest_horizon[0]
            hinfo = (
                f"h0={hz[0]:.2f} hn={hz[-1]:.2f} n={len(hz)}"
                if hz
                else "no-horizon"
            )
            node.get_logger().info(
                f"published Trajectory #{published_count[0]} ({len(out.points)} points, "
                f"v0={v0:.2f} vn={vn:.2f} m/s {hinfo}, "
                f"first=({first.x:.2f}, {first.y:.2f}), "
                f"last=({last.x:.2f}, {last.y:.2f}))"
            )

    node.create_subscription(Odometry, input_odom, on_odom, qos)
    node.create_subscription(Path, input_path, on_path, qos)
    node.create_subscription(Float32MultiArray, input_horizon, on_horizon, qos)
    node.get_logger().info(
        f"{input_path} + {input_odom} + {input_horizon} -> {output_topic} "
        f"(≤{point_count} pts, {extent_cap} m, {byte_budget} B)"
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
