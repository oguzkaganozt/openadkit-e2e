#!/usr/bin/env python3
"""SI-side guard + fallback (Phase 2, sim scope).

Pipeline: follower Control -> guard -> approved Control -> bridge.
The bridge never sees the raw follower output (enforced by routing:
bridge-config.yaml carries only the guard topic; check-guard-routing.sh
fails the run otherwise).

States: INIT -> FRESH <-> COMFORTABLE -> EMERGENCY -> HOLD -> FRESH.
Comfortable may only step down to emergency, never up while the fault
stays; HOLD needs an explicit re-enable pulse plus a cleared fault.

Pure policy lives in GuardPolicy (no ROS) and is unit-tested on
test_guard.py. ROS imports stay in main().
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

# ---- thresholds (derivation in comments) -------------------------------
# SI controller timer runs at 150 ms (~7 Hz); 500 ms is >3 missed cycles.
FOLLOWER_TIMEOUT_MS = 500.0
# Adapter publishes per VP Path (~10 Hz); 1000 ms is ~10 missed inputs,
# matching the adapter's own STALE_INPUT_MS bound.
TRAJ_TIMEOUT_MS = 1000.0
# Bridge publishes odom at 20 Hz; 500 ms is 10 missed samples.
ODOM_TIMEOUT_MS = 500.0
# Babble gates: SI outputs at ~7 Hz, so >40 Hz arrivals is a flood, and
# step jumps beyond these deltas are physically implausible between two
# consecutive follower commands.
BABBLE_RATE_HZ = 40.0
BABBLE_DV_MPS = 8.0
BABBLE_DSTEER_RAD = 0.4
BABBLE_DA_MPS2 = 6.0
# Coupled actuation envelope (never clip steer/accel separately): command
# is rejected when outside 1.2x the (longitudinal, lateral) ellipse.
# Asymmetric by necessity: SI's own emergency decel is -5.0, so a hard
# stop response to a comfortable stop path is legitimate SI behavior, not
# a fault (proven live: symmetric 4.0 turned every comfortable stop into
# an emergency). Runaway throttle stays tightly bound at +4.0.
A_LON_POS_MAX = 4.0
A_LON_NEG_MAX = 6.0
A_LAT_MAX = 3.0
# Frame tag on the guard's own comfortable stop paths. Arrivals carrying
# it are the guard's own DDS loopback and must not count as trajectory
# input (proven live: they faked a "recovery" 0.8 s after a real loss).
OWN_STOP_FRAME_ID = "guard_stop"
ENVELOPE_MARGIN = 1.2
V_MAX_MPS = 40.0
STEER_MAX_RAD = 0.65
# Comfortable activation: the adapter must fall silent (inhibit verified)
# within this long after the inhibit flag, else comfortable failed and the
# guard steps down to emergency.
INHIBIT_VERIFY_MS = 500.0
# Fault-clear hysteresis back to FRESH (frames, evaluated per arrival).
CLEAR_FRAMES = 10
# Emergency brake demand written directly while in EMERGENCY/HOLD.
EMERGENCY_DECEL = -3.0

FRESH = "FRESH"
COMFORTABLE = "COMFORTABLE"
EMERGENCY = "EMERGENCY"
HOLD = "HOLD"
INIT = "INIT"


def _finite(*vals: float) -> bool:
    return all(math.isfinite(v) for v in vals)


@dataclass
class FollowerCmd:
    stamp_ms: float
    velocity: float
    accel: float
    steer: float


@dataclass
class TrajSnapshot:
    stamp_ms: float
    speeds: list[float] = field(default_factory=list)
    # yaw rate (rad/m) over the first metres, for the coupled envelope.
    curvature: float = 0.0
    # True when every speed is (near) zero: a stop path, not a track.
    all_zero: bool = False
    # True when this arrival is the guard's own stop path (frame tag).
    own: bool = False


def is_own_stop(frame_id: str) -> bool:
    return frame_id == OWN_STOP_FRAME_ID


@dataclass
class EgoSnapshot:
    stamp_ms: float
    speed: float


@dataclass
class GuardOutput:
    state: str
    reason: str
    # What the guard publishes as Control this step (None = keep passing
    # the follower command through unchanged).
    override_cmd: FollowerCmd | None = None
    inhibit: bool = True
    publish_stop_traj: bool = False


class GuardPolicy:
    """Pure guard state machine. Times in ms, monotonic."""

    def __init__(self) -> None:
        self.state = INIT
        self.reason = "startup"
        self._last_follower: FollowerCmd | None = None
        self._prev_follower: FollowerCmd | None = None
        self._follower_at = 0.0
        self._arrival_times: list[float] = []
        self._traj: TrajSnapshot | None = None
        self._traj_at = 0.0
        self._ego: EgoSnapshot | None = None
        self._ego_at = 0.0
        self._clear_count = 0
        self._inhibit_since = 0.0
        self._entries = 0
        self.false_stop_entries = 0

    # -- inputs ---------------------------------------------------------
    def on_follower(self, cmd: FollowerCmd) -> None:
        self._prev_follower = self._last_follower
        self._last_follower = cmd
        self._follower_at = cmd.stamp_ms
        self._arrival_times.append(cmd.stamp_ms)
        while self._arrival_times and cmd.stamp_ms - self._arrival_times[0] > 1000.0:
            self._arrival_times.pop(0)

    def on_traj(self, traj: TrajSnapshot, now_ms: float) -> None:
        if traj.own:
            # Own comfortable stop path seen via DDS loopback: never
            # input, never recovery evidence. (Inhibit verification only
            # cares about non-zero arrivals, so dropping these is exact.)
            return
        self._traj = traj
        self._traj_at = now_ms

    def on_odom(self, ego: EgoSnapshot) -> None:
        self._ego = ego
        self._ego_at = ego.stamp_ms

    # -- checks ----------------------------------------------------------
    def _follower_fault(self, now_ms: float) -> str:
        if self._last_follower is None:
            return "no-follower-yet"
        if now_ms - self._follower_at > FOLLOWER_TIMEOUT_MS:
            return "stale-follower"
        cmd = self._last_follower
        if not _finite(cmd.velocity, cmd.accel, cmd.steer):
            return "invalid-follower"
        if len(self._arrival_times) >= 3:
            rate = (len(self._arrival_times) - 1) / max(
                (self._arrival_times[-1] - self._arrival_times[0]) / 1000.0, 1e-6
            )
            if rate > BABBLE_RATE_HZ:
                return "babbling-rate"
        prev = self._prev_follower
        if prev is not None and self._last_follower.stamp_ms > prev.stamp_ms:
            if abs(cmd.velocity - prev.velocity) > BABBLE_DV_MPS:
                return "babbling-jump-v"
            if abs(cmd.steer - prev.steer) > BABBLE_DSTEER_RAD:
                return "babbling-jump-steer"
            if abs(cmd.accel - prev.accel) > BABBLE_DA_MPS2:
                return "babbling-jump-a"
        if not (-0.5 <= cmd.velocity <= V_MAX_MPS):
            return "follower-vel-range"
        if abs(cmd.steer) > STEER_MAX_RAD:
            return "follower-steer-range"
        lat_acc = cmd.velocity * cmd.velocity * self._curvature()
        lon = (
            cmd.accel / A_LON_POS_MAX
            if cmd.accel >= 0.0
            else cmd.accel / A_LON_NEG_MAX
        )
        lat = lat_acc / A_LAT_MAX
        if lon * lon + lat * lat > ENVELOPE_MARGIN * ENVELOPE_MARGIN:
            return "envelope-violation"
        return ""

    def _curvature(self) -> float:
        if self._traj is not None and math.isfinite(self._traj.curvature):
            return self._traj.curvature
        return 0.0

    def _traj_fault(self, now_ms: float) -> str:
        if self._traj is None:
            return "no-traj-yet"
        if now_ms - self._traj_at > TRAJ_TIMEOUT_MS:
            return "stale-traj"
        if not self._traj.speeds or len(self._traj.speeds) < 2:
            return "invalid-traj"
        if not _finite(*self._traj.speeds):
            return "invalid-traj"
        if any(v < -0.5 or v > V_MAX_MPS for v in self._traj.speeds):
            return "traj-range"
        return ""

    def _ego_fault(self, now_ms: float) -> str:
        if self._ego is None:
            return "no-odom-yet"
        if now_ms - self._ego_at > ODOM_TIMEOUT_MS:
            return "stale-odom"
        if not math.isfinite(self._ego.speed) or self._ego.speed < -1.0:
            return "invalid-odom"
        return ""

    # -- step -------------------------------------------------------------
    def step(self, now_ms: float, re_enable: bool = False) -> GuardOutput:
        f_fault = self._follower_fault(now_ms)
        t_fault = self._traj_fault(now_ms)
        e_fault = self._ego_fault(now_ms)
        ego_stopped = self._ego is not None and self._ego.speed < 0.1

        if self.state == INIT:
            if not f_fault and not t_fault and not e_fault:
                return self._enter(FRESH, "inputs-valid", now_ms)
            return GuardOutput(INIT, self.reason or "startup",
                               override_cmd=self._brake_cmd(now_ms),
                               inhibit=True)

        if self.state == FRESH:
            if e_fault:
                return self._enter(EMERGENCY, e_fault, now_ms)
            if f_fault:
                return self._enter(EMERGENCY, f_fault, now_ms)
            if t_fault:
                return self._enter(COMFORTABLE, t_fault, now_ms)
            self._clear_count = 0
            return GuardOutput(FRESH, "ok", inhibit=False)

        if self.state == COMFORTABLE:
            if f_fault or e_fault:
                # The fault is the follower itself (or ego): skip to
                # emergency, and a failed inhibit verifies the same way.
                return self._enter(EMERGENCY, f_fault or e_fault, now_ms)
            if t_fault:
                if not self._inhibit_verified(now_ms):
                    if now_ms - self._inhibit_since > INHIBIT_VERIFY_MS:
                        return self._enter(EMERGENCY, "inhibit-failed", now_ms)
            if not t_fault:
                self._clear_count += 1
                if self._clear_count >= CLEAR_FRAMES:
                    return self._enter(FRESH, "traj-recovered", now_ms)
            else:
                self._clear_count = 0
            if ego_stopped:
                return self._enter(HOLD, "comfortable-stop", now_ms)
            return GuardOutput(COMFORTABLE, self.reason,
                               inhibit=True, publish_stop_traj=True)

        if self.state == EMERGENCY:
            if ego_stopped:
                return self._enter(HOLD, "emergency-stop", now_ms)
            return GuardOutput(EMERGENCY, self.reason,
                               override_cmd=self._brake_cmd(now_ms),
                               inhibit=True)

        if self.state == HOLD:
            if re_enable and not f_fault and not t_fault and not e_fault:
                return self._enter(FRESH, "re-enabled", now_ms)
            if re_enable:
                self.reason = "re-enable-refused-fault-present"
            return GuardOutput(HOLD, self.reason, inhibit=True,
                               publish_stop_traj=True,
                               override_cmd=self._brake_cmd(now_ms))

        raise AssertionError("unknown guard state " + self.state)

    # -- helpers ------------------------------------------------------------
    def _enter(self, state: str, reason: str, now_ms: float) -> GuardOutput:
        self.state = state
        self.reason = reason
        self._clear_count = 0
        if state in (COMFORTABLE, EMERGENCY):
            self._entries += 1
            self.false_stop_entries += 1
            self._inhibit_since = now_ms
        out = self.step_after_enter(now_ms)
        return out

    def step_after_enter(self, now_ms: float) -> GuardOutput:
        if self.state == FRESH:
            return GuardOutput(FRESH, self.reason, inhibit=False)
        if self.state == COMFORTABLE:
            return GuardOutput(COMFORTABLE, self.reason,
                               inhibit=True, publish_stop_traj=True)
        if self.state in (EMERGENCY, HOLD):
            return GuardOutput(HOLD if self.state == HOLD else EMERGENCY,
                               self.reason, inhibit=True,
                               publish_stop_traj=self.state == HOLD,
                               override_cmd=self._brake_cmd(now_ms))
        return GuardOutput(INIT, self.reason,
                           override_cmd=self._brake_cmd(now_ms), inhibit=True)

    def _inhibit_verified(self, now_ms: float) -> bool:
        # Verified while no TRACK (non-zero) trajectory arrives: the only
        # publisher left after a good inhibit is the guard's own stop path.
        # (Own stop pubs are all-zero, so they never fail this check.)
        if self._traj is None:
            return True
        if now_ms - self._traj_at > TRAJ_TIMEOUT_MS:
            return True
        return self._traj.all_zero

    def _brake_cmd(self, now_ms: float) -> FollowerCmd:
        steer = 0.0
        if self._last_follower is not None and math.isfinite(
            self._last_follower.steer
        ):
            steer = self._last_follower.steer
        return FollowerCmd(
            stamp_ms=now_ms, velocity=0.0, accel=EMERGENCY_DECEL, steer=steer
        )


def main(args=None) -> None:
    import time as time_mod

    import rclpy
    from autoware_control_msgs.msg import Control
    from autoware_planning_msgs.msg import Trajectory
    from autoware_planning_msgs.msg import TrajectoryPoint as TrajMsg
    from builtin_interfaces.msg import Duration
    from nav_msgs.msg import Odometry
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from std_msgs.msg import Bool, String

    rclpy.init(args=args)
    node = Node("si_guard")

    qos = QoSProfile(
        depth=1,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    qos_latched = QoSProfile(
        depth=1,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )

    policy = GuardPolicy()
    approved_pub = node.create_publisher(Control, "/control/guard/control_cmd", qos)
    stop_traj_pub = node.create_publisher(
        Trajectory, "/planning/scenario_planning/trajectory", qos
    )
    inhibit_pub = node.create_publisher(Bool, "/guard/inhibit_trajectory", qos_latched)
    state_pub = node.create_publisher(String, "/guard/state", qos)
    last_traj_geom: list = [None]
    last_state: list[str] = [""]
    last_log_at = [0.0]
    re_enable_flag = [False]
    want_cmd: list[FollowerCmd | None] = [None]

    def ms() -> float:
        return time_mod.monotonic() * 1000.0

    def on_follower(msg: Control) -> None:
        cmd = FollowerCmd(
            stamp_ms=ms(),
            velocity=float(msg.longitudinal.velocity),
            accel=float(msg.longitudinal.acceleration),
            steer=float(msg.lateral.steering_tire_angle),
        )
        want_cmd[0] = cmd
        policy.on_follower(cmd)
        tick(publish=True)

    def on_traj(msg: Trajectory) -> None:
        if is_own_stop(msg.header.frame_id):
            return
        speeds = [float(p.longitudinal_velocity_mps) for p in msg.points]
        curvature = 0.0
        if len(msg.points) >= 3:
            try:
                import math as _m

                yaws = []
                for p in msg.points[:5]:
                    q = p.pose.orientation
                    yaws.append(
                        _m.atan2(
                            2.0 * (q.w * q.z + q.x * q.y),
                            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
                        )
                    )
                dydx = 0.0
                dist = 0.0
                for i in range(1, len(msg.points[:5])):
                    a = msg.points[i - 1].pose.position
                    b = msg.points[i].pose.position
                    dist += _m.hypot(b.x - a.x, b.y - a.y)
                if dist > 0.5:
                    dyaw = (yaws[-1] - yaws[0] + _m.pi) % (2.0 * _m.pi) - _m.pi
                    curvature = dyaw / dist
            except Exception:
                curvature = 0.0
        policy.on_traj(
            TrajSnapshot(
                stamp_ms=ms(),
                speeds=speeds,
                curvature=curvature,
                all_zero=bool(speeds) and all(abs(v) < 0.05 for v in speeds),
            ),
            ms(),
        )
        if speeds and not all(abs(v) < 0.05 for v in speeds):
            # Plain geometry (no ROS aliasing): rebuilt as a zero-speed
            # stop path if comfortable needs one.
            geom = [
                (
                    p.pose.position.x,
                    p.pose.position.y,
                    p.pose.position.z,
                    p.pose.orientation.x,
                    p.pose.orientation.y,
                    p.pose.orientation.z,
                    p.pose.orientation.w,
                )
                for p in msg.points
            ]
            last_traj_geom.append(geom)
            if len(last_traj_geom) > 4:
                last_traj_geom.pop(0)

    def on_odom(msg: Odometry) -> None:
        policy.on_odom(
            EgoSnapshot(stamp_ms=ms(), speed=float(msg.twist.twist.linear.x))
        )

    def on_re_enable(msg: Bool) -> None:
        if msg.data:
            re_enable_flag[0] = True

    def publish_stop_traj() -> None:
        src = None
        for cand in reversed(last_traj_geom):
            if cand is not None:
                src = cand
                break
        out = Trajectory()
        out.header.stamp = node.get_clock().now().to_msg()
        out.header.frame_id = OWN_STOP_FRAME_ID
        if src is not None:
            for i, (x, y, z, qx, qy, qz, qw) in enumerate(src):
                tp = TrajMsg()
                tp.pose.position.x = x
                tp.pose.position.y = y
                tp.pose.position.z = z
                tp.pose.orientation.x = qx
                tp.pose.orientation.y = qy
                tp.pose.orientation.z = qz
                tp.pose.orientation.w = qw
                tp.longitudinal_velocity_mps = 0.0
                tp.acceleration_mps2 = -1.0
                tp.time_from_start = Duration(sec=0, nanosec=250_000_000 * i)
                out.points.append(tp)
        stop_traj_pub.publish(out)

    def tick(publish: bool = False) -> None:
        now = ms()
        out = policy.step(now, re_enable=re_enable_flag[0])
        re_enable_flag[0] = False
        cmd = want_cmd[0]
        if out.override_cmd is not None:
            c = Control()
            c.stamp = node.get_clock().now().to_msg()
            c.longitudinal.velocity = out.override_cmd.velocity
            c.longitudinal.acceleration = out.override_cmd.accel
            c.lateral.steering_tire_angle = out.override_cmd.steer
            approved_pub.publish(c)
            applied = (
                out.override_cmd.velocity,
                out.override_cmd.accel,
                out.override_cmd.steer,
            )
        elif cmd is not None:
            c = Control()
            c.stamp = node.get_clock().now().to_msg()
            c.longitudinal.velocity = cmd.velocity
            c.longitudinal.acceleration = cmd.accel
            c.lateral.steering_tire_angle = cmd.steer
            approved_pub.publish(c)
            applied = (cmd.velocity, cmd.accel, cmd.steer)
        else:
            applied = (float("nan"), float("nan"), float("nan"))
        if out.inhibit:
            inhibit_pub.publish(Bool(data=True))
        else:
            inhibit_pub.publish(Bool(data=False))
        if out.publish_stop_traj:
            publish_stop_traj()
        if out.state != last_state[0]:
            last_state[0] = out.state
            node.get_logger().warn(
                "guard -> %s reason=%s" % (out.state, out.reason)
            )
            state_pub.publish(String(data=out.state))
        if publish or now - last_log_at[0] > 1000.0:
            last_log_at[0] = now
            w = want_cmd[0]
            node.get_logger().info(
                "guard state=%s reason=%s want_v=%s want_a=%s "
                "applied_v=%.2f applied_a=%.2f entries=%d" % (
                    out.state,
                    out.reason,
                    ("%.2f" % w.velocity) if w else "NA",
                    ("%.2f" % w.accel) if w else "NA",
                    applied[0] if math.isfinite(applied[0]) else -1.0,
                    applied[1] if math.isfinite(applied[1]) else -99.0,
                    policy.false_stop_entries,
                )
            )

    node.create_subscription(Control, "/control/trajectory_follower/control_cmd", on_follower, qos)
    node.create_subscription(Trajectory, "/planning/scenario_planning/trajectory", on_traj, qos)
    node.create_subscription(Odometry, "/localization/kinematic_state", on_odom, qos)
    node.create_subscription(Bool, "/guard/re_enable", on_re_enable, qos)
    node.create_timer(0.2, lambda: tick(publish=False))
    node.get_logger().info("si_guard: follower Control -> approved Control")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
