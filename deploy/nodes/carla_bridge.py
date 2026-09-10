#!/usr/bin/env python3
"""SI DDS ↔ CARLA RPC. VP steering_cmd is not used."""

import math
import threading
import time
from array import array
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import carla
import cv2
import numpy as np
import rclpy
from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import SteeringReport
from geometry_msgs.msg import AccelWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64


def _quat_from_yaw(yaw: float):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


STOP_SPEED_MPS = 0.05
CRUISE_THROTTLE_PER_MPS = 0.12
SPEED_P_GAIN = 0.15
ACCEL_THROTTLE_GAIN = 0.08
BRAKE_DECEL_GAIN = 0.25
MAX_THROTTLE = 0.5
CONTROL_TIMEOUT_SEC = 0.5
STALE_SAMPLE_TIMEOUT_SEC = 0.2


def carla_longitudinal(cmd_v: float, cmd_a: float, actual_v: float) -> tuple[float, float]:
    if cmd_v <= STOP_SPEED_MPS and cmd_a <= 0.0:
        return 0.0, 0.4
    speed_error = cmd_v - actual_v
    if speed_error < -0.4 or (cmd_a < 0.0 and speed_error < 0.0):
        return 0.0, min(1.0, max(-cmd_a * BRAKE_DECEL_GAIN, -speed_error * 0.5))
    throttle = (
        CRUISE_THROTTLE_PER_MPS * max(cmd_v, 0.0)
        + SPEED_P_GAIN * speed_error
        + ACCEL_THROTTLE_GAIN * max(cmd_a, 0.0)
    )
    return max(0.0, min(MAX_THROTTLE, throttle)), 0.0


_jpeg = None
_jpeg_lock = threading.Lock()


class _MjpegHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        return

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(
                b"<html><body style='margin:0;background:#111'>"
                b"<img src='/stream' style='width:100%'></body></html>"
            )
            return
        if self.path != "/stream":
            self.send_error(404)
            return
        self.send_response(200)
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        try:
            while True:
                with _jpeg_lock:
                    frame = _jpeg
                if frame:
                    self.wfile.write(
                        b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n"
                        % len(frame)
                    )
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
                    self.wfile.flush()
                time.sleep(0.05)
        except (BrokenPipeError, ConnectionResetError):
            return


class CarlaBridge(Node):
    def __init__(self):
        super().__init__("carla_bridge")
        self.declare_parameter("carla_host", "127.0.0.1")
        self.declare_parameter("carla_port", 2000)
        self.declare_parameter("role_name", "hero")
        self.declare_parameter("camera_role", "main_cam")
        host = self.get_parameter("carla_host").value
        port = int(self.get_parameter("carla_port").value)
        self._role = self.get_parameter("role_name").value
        self._camera_role = self.get_parameter("camera_role").value

        self._client = carla.Client(host, port)
        self._client.set_timeout(2.0)
        self._world = None
        self._vehicle = None
        self._camera = None
        self._last_steer = 0.0
        self._control_count = 0
        self._http_server = None
        self._lock = threading.Lock()
        self._stop = False
        self._pending_ctrl = None
        self._pending_ctrl_at = 0.0
        self._ros_image = None
        self._sample = None
        self._actual_v = 0.0
        self._max_steer = None
        self._preview_frame = None
        self._camera_count = 0
        self._camera_started = time.monotonic()

        img_qos = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.odom_pub = self.create_publisher(
            Odometry, "/localization/kinematic_state", 1
        )
        self.accel_pub = self.create_publisher(
            AccelWithCovarianceStamped, "/localization/acceleration", 1
        )
        self.steer_pub = self.create_publisher(
            SteeringReport, "/vehicle/status/steering_status", 1
        )
        self.speed_pub = self.create_publisher(Float64, "/vehicle/speed", 1)
        self.image_pub = self.create_publisher(
            Image, "/carla/hero/main_cam/image", img_qos
        )
        self.create_subscription(
            Control, "/control/trajectory_follower/control_cmd", self._on_control, 1
        )
        self.create_timer(0.05, self._on_timer)
        threading.Thread(target=self._http, daemon=True).start()
        threading.Thread(target=self._preview_loop, daemon=True).start()
        threading.Thread(target=self._image_pub_loop, daemon=True).start()
        threading.Thread(target=self._carla_loop, daemon=True).start()
        self.get_logger().info("carla_bridge: CARLA RPC %s:%s http://0.0.0.0:8090/" % (host, port))

    def _connect(self):
        if self._world is not None:
            return True
        try:
            self._world = self._client.get_world()
            return True
        except Exception as exc:
            self.get_logger().warn("waiting for CARLA: %s" % exc)
            return False

    def _find_actors(self):
        if self._world is None:
            return
        if self._vehicle is None or not self._vehicle.is_alive:
            self._vehicle = None
            self._camera = None
            for actor in self._world.get_actors().filter("vehicle.*"):
                if actor.attributes.get("role_name") == self._role:
                    self._vehicle = actor
                    self.get_logger().info("hero id=%s" % actor.id)
                    break
        if self._vehicle is None:
            return
        if self._camera is None:
            for actor in self._world.get_actors().filter("sensor.camera.rgb"):
                if actor.attributes.get("role_name") == self._camera_role:
                    self._camera = actor
                    break
            if self._camera is None:
                # Safety net when the scenario did not spawn the camera. Values must
                # match deploy/config/carla-rig.json, converted to the CARLA frame
                # the same way scenario.py does (y/pitch/yaw negated).
                bp = self._world.get_blueprint_library().find("sensor.camera.rgb")
                bp.set_attribute("image_size_x", "1920")
                bp.set_attribute("image_size_y", "1280")
                bp.set_attribute("fov", "50")
                bp.set_attribute("sensor_tick", "0.1")
                tf = carla.Transform(
                    carla.Location(x=1.544, y=0.0243, z=2.116),
                    carla.Rotation(pitch=-0.11, yaw=-0.23, roll=-0.1),
                )
                try:
                    self._camera = self._world.spawn_actor(
                        bp, tf, attach_to=self._vehicle
                    )
                except Exception as exc:
                    self._camera = None
                    self.get_logger().warn("camera spawn failed: %s" % exc)
                    return
            try:
                self._camera.listen(self._on_camera)
                self.get_logger().info(
                    "camera id=%s %sx%s"
                    % (
                        self._camera.id,
                        self._camera.attributes.get("image_size_x"),
                        self._camera.attributes.get("image_size_y"),
                    )
                )
            except Exception as exc:
                self._camera = None
                self.get_logger().warn("camera listen failed: %s" % exc)

    def _on_timer(self):
        with self._lock:
            sample = self._sample
        if sample is None:
            return
        if time.monotonic() - sample.get("t_monotonic", 0.0) > STALE_SAMPLE_TIMEOUT_SEC:
            return
        last_steer = sample.get("steer", self._last_steer)
        qx, qy, qz, qw = _quat_from_yaw(sample["yaw"])
        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = sample["x"]
        odom.pose.pose.position.y = sample["y"]
        odom.pose.pose.position.z = sample["z"]
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = sample["vx"]
        odom.twist.twist.linear.y = sample["vy"]
        odom.twist.twist.linear.z = sample["vz"]
        self.odom_pub.publish(odom)
        self.speed_pub.publish(Float64(data=sample["speed"]))

        accel = AccelWithCovarianceStamped()
        accel.header.stamp = stamp
        accel.header.frame_id = "base_link"
        accel.accel.accel.linear.x = sample["ax"]
        accel.accel.accel.linear.y = sample["ay"]
        accel.accel.accel.linear.z = sample["az"]
        self.accel_pub.publish(accel)

        steer = SteeringReport()
        steer.stamp = stamp
        steer.steering_tire_angle = last_steer
        self.steer_pub.publish(steer)

    def _carla_loop(self):
        while not self._stop:
            try:
                if not self._connect():
                    time.sleep(0.2)
                    continue
                self._find_actors()
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
                with self._lock:
                    ctrl = self._pending_ctrl
                    ctrl_at = self._pending_ctrl_at
                if ctrl is not None:
                    if time.monotonic() - ctrl_at > CONTROL_TIMEOUT_SEC:
                        ctrl = carla.VehicleControl(throttle=0.0, brake=0.4, steer=0.0)
                    vehicle.apply_control(ctrl)
                t = vehicle.get_transform()
                vel = vehicle.get_velocity()
                acc = vehicle.get_acceleration()
                yaw = -math.radians(t.rotation.yaw)
                cos_y = math.cos(yaw)
                sin_y = math.sin(yaw)
                vel_x = float(vel.x)
                vel_y = float(-vel.y)
                acc_x = float(acc.x)
                acc_y = float(-acc.y)
                actual_v = cos_y * vel_x + sin_y * vel_y
                sample = {
                    "t_monotonic": time.monotonic(),
                    "x": float(t.location.x),
                    "y": float(-t.location.y),
                    "z": float(t.location.z),
                    "yaw": yaw,
                    "yaw_deg": float(t.rotation.yaw),
                    "vx": actual_v,
                    "vy": -sin_y * vel_x + cos_y * vel_y,
                    "vz": float(vel.z),
                    "ax": cos_y * acc_x + sin_y * acc_y,
                    "ay": -sin_y * acc_x + cos_y * acc_y,
                    "az": float(acc.z),
                    "speed": math.hypot(vel.x, vel.y),
                    "steer": self._measured_steer(vehicle),
                }
                with self._lock:
                    self._sample = sample
                    self._actual_v = actual_v
            except RuntimeError as exc:
                self._vehicle = None
                self._camera = None
                self._max_steer = None
                self.get_logger().warn("hero became unavailable: %s" % exc)
            time.sleep(0.05)

    def _measured_steer(self, vehicle) -> float:
        try:
            deg = vehicle.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)
            return -math.radians(float(deg))
        except Exception:
            return self._last_steer

    def _on_camera(self, image):
        try:
            arr = np.frombuffer(image.raw_data, dtype=np.uint8)
            arr = arr.reshape((image.height, image.width, 4))
            bgr = np.ascontiguousarray(arr[:, :, :3])
            with self._lock:
                self._ros_image = (image.width, image.height, bgr)
                self._preview_frame = bgr
            self._camera_count += 1
            if self._camera_count == 1 or self._camera_count % 50 == 0:
                elapsed = time.monotonic() - self._camera_started
                self.get_logger().info(
                    "camera frame #%d (%.2f Hz)"
                    % (self._camera_count, self._camera_count / elapsed)
                )
        except Exception as exc:
            self.get_logger().error("camera: %s" % exc)

    def _image_pub_loop(self):
        while not self._stop:
            with self._lock:
                item = self._ros_image
                self._ros_image = None
            if item is None:
                time.sleep(0.005)
                continue
            width, height, bgr = item
            try:
                msg = Image()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "hero/main_cam"
                msg.height = height
                msg.width = width
                msg.encoding = "bgr8"
                msg.is_bigendian = 0
                msg.step = width * 3
                data = array("B")
                data.frombytes(bgr)
                msg.data = data
                self.image_pub.publish(msg)
            except Exception as exc:
                self.get_logger().error("image publish: %s" % exc)

    def _preview_loop(self):
        global _jpeg
        while not self._stop:
            with self._lock:
                frame = self._preview_frame
                self._preview_frame = None
            if frame is None:
                time.sleep(0.02)
                continue
            try:
                small = cv2.resize(frame, (640, 427))
                ok, buf = cv2.imencode(
                    ".jpg", small, [int(cv2.IMWRITE_JPEG_QUALITY), 45]
                )
                if ok:
                    with _jpeg_lock:
                        _jpeg = buf.tobytes()
            except Exception as exc:
                self.get_logger().error("preview: %s" % exc)

    def _http(self):
        self._http_server = ThreadingHTTPServer(("0.0.0.0", 8090), _MjpegHandler)
        self._http_server.daemon_threads = True
        self._http_server.serve_forever()

    def _on_control(self, msg: Control):
        self._last_steer = float(msg.lateral.steering_tire_angle)
        with self._lock:
            sample = self._sample
            actual_v = self._actual_v
            max_steer = self._max_steer or 1.0
        if sample is None:
            return
        ctrl = carla.VehicleControl()
        # Autoware uses positive-left; CARLA's normalized input is positive-right.
        ctrl.steer = max(-1.0, min(1.0, -self._last_steer / max_steer))
        acc = float(msg.longitudinal.acceleration)
        vel = float(msg.longitudinal.velocity)
        ctrl.throttle, ctrl.brake = carla_longitudinal(vel, acc, actual_v)
        ctrl.hand_brake = False
        ctrl.manual_gear_shift = False
        with self._lock:
            self._pending_ctrl = ctrl
            self._pending_ctrl_at = time.monotonic()
        self._control_count += 1
        if self._control_count == 1 or self._control_count % 100 == 0:
            pose_x = sample["x"] if sample else 0.0
            pose_y = -sample["y"] if sample else 0.0
            pose_yaw = sample["yaw_deg"] if sample else 0.0
            self.get_logger().info(
                "applied control #%d: cmd_v=%.2f actual_v=%.2f cmd_a=%.2f "
                "tire=%.3f carla=%.3f throttle=%.3f brake=%.3f "
                "pose=(%.2f, %.2f, %.1fdeg)"
                % (
                    self._control_count,
                    vel,
                    actual_v,
                    acc,
                    self._last_steer,
                    ctrl.steer,
                    ctrl.throttle,
                    ctrl.brake,
                    pose_x,
                    pose_y,
                    pose_yaw,
                )
            )

    def destroy_node(self):
        self._stop = True
        if self._http_server is not None:
            self._http_server.shutdown()
            self._http_server.server_close()
        if self._camera is not None:
            try:
                self._camera.stop()
            except Exception:
                pass
        super().destroy_node()


def main():
    rclpy.init()
    node = CarlaBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
