#!/usr/bin/env python3
"""Single CARLA control writer for the E2E rig (E2E #2).

The Safety Island's ApprovedRequest is the only actuation input of this
process, and this process is the only writer of `vehicle.apply_control()` in
the rig:

  NORMAL  -> actuate the approved payload of this request
  SI_STOP -> actuate the explicit stop payload SI computed (target speed 0,
             signed negative acceleration); the actuator never decides the
             stop itself
  HOLD    -> keep actuating the previous approved payload
  unknown -> treated as HOLD, per the message contract

It deliberately has no watchdog brake, no speed target of its own, no fault
latch and no mode/source switching: if SI stops producing, CARLA keeps the
last applied control and the rig offers no stop guarantee. The 500 ms gate
(fault detection -> CARLA-applied stop) is instrumented with wall-clock GATE
lines for the measurement script (deploy/tools/stop-gate-test.sh).

Runs on the SI DDS domain in the adapter image (carries safety_island_msgs):

    ROS_DOMAIN_ID=2 python3 /opt/nodes/carla_actuator.py
"""

import math
import threading
import time

import carla
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from safety_island_msgs.msg import ApprovedRequest

DECISION_NORMAL = 0
DECISION_STOP = 1
DECISION_HOLD = 2

# Pedal mapping of an approved (velocity, acceleration) request; the same
# deterministic translation the bridge used, kept identical so the actuator
# swap changes authority, not vehicle behavior.
STOP_SPEED_MPS = 0.05
CRUISE_THROTTLE_PER_MPS = 0.12
SPEED_P_GAIN = 0.15
ACCEL_THROTTLE_GAIN = 0.08
BRAKE_DECEL_GAIN = 0.25
MAX_THROTTLE = 0.5
APPLIED_LOG_INTERVAL = 100
SILENCE_LOG_SEC = 5.0


def carla_longitudinal(cmd_v: float, cmd_a: float, actual_v: float) -> tuple[float, float]:
    if cmd_v <= STOP_SPEED_MPS and cmd_a <= 0.0:
        return 0.0, 0.4
    speed_error = cmd_v - actual_v
    # An explicit SI decel demand (<= -0.5 m/s^2) brakes even before the
    # velocity error turns negative; the threshold keeps regulation chatter
    # on throttle.
    if speed_error < -0.4 or cmd_a <= -0.5 or (cmd_a < 0.0 and speed_error < 0.0):
        return 0.0, min(1.0, max(-cmd_a * BRAKE_DECEL_GAIN, -speed_error * 0.5))
    throttle = (
        CRUISE_THROTTLE_PER_MPS * max(cmd_v, 0.0)
        + SPEED_P_GAIN * speed_error
        + ACCEL_THROTTLE_GAIN * max(cmd_a, 0.0)
    )
    return max(0.0, min(MAX_THROTTLE, throttle)), 0.0


class CarlaActuator(Node):
    def __init__(self):
        super().__init__("carla_actuator")
        self.declare_parameter("carla_host", "127.0.0.1")
        self.declare_parameter("carla_port", 2000)
        self.declare_parameter("role_name", "hero")
        host = self.get_parameter("carla_host").value
        port = int(self.get_parameter("carla_port").value)
        self._role = self.get_parameter("role_name").value

        self._client = carla.Client(host, port)
        self._client.set_timeout(2.0)
        self._world = None
        self._vehicle = None
        self._max_steer = None
        self._stop = False
        self._lock = threading.Lock()

        # Latest approved payload; None until the first NORMAL/SI_STOP sample.
        self._payload = None
        self._actual_v = 0.0
        self._applied = 0
        self._last_decision = None

        self._last_msg_mono = None
        self._session = None
        self._expected_seq = None
        # Stop-gate instrumentation: armed by an SI_STOP decision, discharged
        # by the first applied stop frame of that episode.
        self._stop_waiting = False
        self._stop_recv_wall_ns = None
        self._stop_decision = None
        self._stop_armed_fault = None

        qos = QoSProfile(
            depth=10,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            ApprovedRequest,
            "/control/safety_island/approved_request",
            self._on_approved,
            qos,
        )
        threading.Thread(target=self._carla_loop, daemon=True).start()
        self.create_timer(SILENCE_LOG_SEC, self._silence_check)
        self.get_logger().info(
            "carla_actuator: sole CARLA control writer (ApprovedRequest only)"
        )

    def _on_approved(self, msg: ApprovedRequest) -> None:
        self._last_msg_mono = time.monotonic()
        raw_decision = int(msg.decision)
        decision = raw_decision if raw_decision in (0, 1, 2) else DECISION_HOLD
        session = int(msg.session)
        seq = int(msg.output_sequence)
        if self._session != session:
            self.get_logger().info(
                "SI session %s -> %d; sequence tracking reset"
                % (self._session, session)
            )
            self._session = session
            self._expected_seq = None
        if self._expected_seq is not None:
            if seq > self._expected_seq + 1:
                self.get_logger().warn(
                    "ApprovedRequest sequence gap: %d -> %d (%d missing)"
                    % (self._expected_seq, seq, seq - self._expected_seq - 1)
                )
            elif seq < self._expected_seq:
                self.get_logger().warn(
                    "ApprovedRequest sequence backwards: %d -> %d"
                    % (self._expected_seq, seq)
                )
        self._expected_seq = max(seq, self._expected_seq or 0)

        if decision != self._last_decision:
            self.get_logger().info(
                "SI control state: %s (session=%d seq=%d fault=%d)"
                % ({0: "NORMAL", 1: "SI_STOP", 2: "HOLD"}[decision],
                   session, seq, int(msg.fault_id))
            )
            self._last_decision = decision

        if decision == DECISION_HOLD:
            return

        payload = {
            "decision": decision,
            "mode": int(msg.mode),
            "selected_source": int(msg.selected_source),
            "session": session,
            "seq": seq,
            "fault_id": int(msg.fault_id),
            "velocity": float(msg.control.longitudinal.velocity),
            "acceleration": float(msg.control.longitudinal.acceleration),
            "tire": float(msg.control.lateral.steering_tire_angle),
        }
        with self._lock:
            self._payload = payload

        if decision == DECISION_STOP:
            fault_key = (session, payload["fault_id"])
            if fault_key != self._stop_armed_fault:
                # One STOP_RECV/STOP_APPLIED pair per fault episode: SI keeps
                # publishing SI_STOP at its cycle rate while latched.
                self._stop_armed_fault = fault_key
                self._stop_waiting = True
                self._stop_recv_wall_ns = time.time_ns()
                self._stop_decision = (
                    "session=%d seq=%d mode=%d source=%d fault=%d"
                    % (session, seq, payload["mode"], payload["selected_source"],
                       payload["fault_id"])
                )
                self.get_logger().warn(
                    "GATE STOP_RECV %s wall_ns=%d"
                    % (self._stop_decision, self._stop_recv_wall_ns)
                )
        else:
            self._stop_armed_fault = None

    def _silence_check(self) -> None:
        if self._last_msg_mono is None:
            return
        age = time.monotonic() - self._last_msg_mono
        if age > SILENCE_LOG_SEC:
            # Visibility only: no timeout brake, SI owns stops.
            self.get_logger().warn(
                "no ApprovedRequest for %.1f s; holding the last applied control"
                % age
            )

    def _connect(self) -> bool:
        if self._world is not None:
            return True
        try:
            self._world = self._client.get_world()
            return True
        except Exception as exc:
            self.get_logger().warn("waiting for CARLA: %s" % exc)
            return False

    def _find_vehicle(self) -> None:
        if self._world is None:
            return
        if self._vehicle is not None and self._vehicle.is_alive:
            return
        self._vehicle = None
        self._max_steer = None
        for actor in self._world.get_actors().filter("vehicle.*"):
            if actor.attributes.get("role_name") == self._role:
                self._vehicle = actor
                self.get_logger().info("hero id=%s" % actor.id)
                break

    def _carla_loop(self) -> None:
        while not self._stop:
            try:
                if not self._connect():
                    time.sleep(0.2)
                    continue
                self._find_vehicle()
                vehicle = self._vehicle
                if vehicle is None or not vehicle.is_alive:
                    time.sleep(0.1)
                    continue
                if self._max_steer is None:
                    physics = vehicle.get_physics_control()
                    max_steer = (
                        math.radians(physics.wheels[0].max_steer_angle)
                        if physics.wheels
                        else 1.0
                    )
                    self._max_steer = max_steer if max_steer > 1e-6 else 1.0

                snapshot = self._world.get_snapshot()
                frame = int(snapshot.frame)
                t = vehicle.get_transform()
                vel = vehicle.get_velocity()
                if int(self._world.get_snapshot().frame) != frame:
                    time.sleep(0.005)
                    continue
                yaw = -math.radians(t.rotation.yaw)
                vel_x = float(vel.x)
                vel_y = float(-vel.y)
                actual_v = math.cos(yaw) * vel_x + math.sin(yaw) * vel_y
                with self._lock:
                    payload = self._payload
                    self._actual_v = actual_v
                if payload is None:
                    # No approved request has ever arrived: write nothing.
                    time.sleep(0.01)
                    continue

                ctrl = carla.VehicleControl()
                ctrl.steer = max(
                    -1.0, min(1.0, -payload["tire"] / (self._max_steer or 1.0))
                )
                ctrl.throttle, ctrl.brake = carla_longitudinal(
                    payload["velocity"], payload["acceleration"], actual_v
                )
                ctrl.hand_brake = False
                ctrl.manual_gear_shift = False
                vehicle.apply_control(ctrl)
                self._applied += 1
                if self._applied == 1 or self._applied % APPLIED_LOG_INTERVAL == 0:
                    self.get_logger().info(
                        "applied control #%d: decision=%d session=%d seq=%d "
                        "source=%d fault=%d cmd_v=%.2f cmd_a=%.2f actual_v=%.2f "
                        "tire=%.3f carla=%.3f throttle=%.3f brake=%.3f frame=%d"
                        % (
                            self._applied,
                            payload["decision"],
                            payload["session"],
                            payload["seq"],
                            payload["selected_source"],
                            payload["fault_id"],
                            payload["velocity"],
                            payload["acceleration"],
                            actual_v,
                            payload["tire"],
                            ctrl.steer,
                            ctrl.throttle,
                            ctrl.brake,
                            frame,
                        )
                    )
                # The applied-stop gate frame: first CARLA frame this process
                # applies the SI stop payload on.
                if self._stop_waiting and ctrl.brake > 0.05 and ctrl.throttle <= 0.01:
                    self._stop_waiting = False
                    self.get_logger().warn(
                        "GATE STOP_APPLIED %s frame=%d wall_ns=%d throttle=%.3f brake=%.3f"
                        % (self._stop_decision, frame, time.time_ns(),
                           ctrl.throttle, ctrl.brake)
                    )
            except RuntimeError as exc:
                self._vehicle = None
                self._max_steer = None
                self.get_logger().warn("hero became unavailable: %s" % exc)
            time.sleep(0.01)

    def destroy_node(self) -> None:
        self._stop = True
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = CarlaActuator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()