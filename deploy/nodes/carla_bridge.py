#!/usr/bin/env python3
"""CARLA telemetry -> SI DDS: camera, ground-truth odometry and /clock only.

Control application was moved to the single-writer carla_actuator.py (E2E #2):
this bridge no longer subscribes to any control topic and never writes
vehicle.apply_control(); it cannot drive or brake the car."""

import math
import threading
import time
from array import array
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import carla
import cv2
import numpy as np
import os
import rclpy
from autoware_vehicle_msgs.msg import SteeringReport
from builtin_interfaces.msg import Time
from geometry_msgs.msg import AccelWithCovarianceStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64


def _quat_from_yaw(yaw: float):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def _sim_stamp(seconds: float) -> Time:
    """CARLA simulated seconds (episode elapsed) as a ROS time message.

    Source identity, not host time: the consumers compare camera and ego
    samples against this value to join the same simulator frame.
    """
    sec = int(seconds)
    nanosec = int(round((seconds - sec) * 1e9))
    if nanosec >= 1000000000:
        sec += 1
        nanosec -= 1000000000
    stamp = Time()
    stamp.sec = sec
    stamp.nanosec = nanosec
    return stamp


# Stream watchdog (s): if the world ticks but no camera frame arrives for
# this long, the sensor session was purged server-side (e.g. another client
# changed the world); the CARLA client would otherwise keep requesting the
# dead stream id until the server's RPC starves. Re-listen to get a fresh
# session instead.
STREAM_WATCHDOG_SEC = 5.0


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
        self._http_server = None
        self._lock = threading.Lock()
        self._stop = False
        self._ros_image = None
        self._sample = None
        self._last_frame = None
        self._last_frame_at = 0.0
        self._preview_frame = None
        self._camera_count = 0
        self._camera_started = time.monotonic()
        # 1 Hz run-evidence snapshots (user asked: a picture every second
        # next to the logs). Host-mounted at /tmp/snaps via compose.
        self._snap_dir = os.environ.get("SNAP_DIR", "/tmp/snaps")
        self._snap_period = float(os.environ.get("SNAP_PERIOD_SEC", "1.0"))
        self._snap_keep = int(os.environ.get("SNAP_KEEP", "900"))
        try:
            os.makedirs(self._snap_dir, exist_ok=True)
        except Exception:
            self._snap_dir = ""

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
        # CARLA simulation clock for consumers that run with use_sim_time
        # (the Autoware planning configuration does). Stamped with the same
        # source-identity time as the ego topics.
        self.clock_pub = self.create_publisher(Clock, "/clock", 1)
        threading.Thread(target=self._http, daemon=True).start()
        threading.Thread(target=self._preview_loop, daemon=True).start()
        threading.Thread(target=self._snap_loop, daemon=True).start()
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
        if self._camera is not None and not self._camera.is_alive:
            # The camera died with a world change; drop it so it is re-found
            # (or spawned) below instead of listening on a dead actor.
            self._camera = None
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
                self._last_frame_at = time.monotonic()
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

    def _publish_sample(self, sample):
        qx, qy, qz, qw = _quat_from_yaw(sample["yaw"])
        # Source identity: stamp with the CARLA simulated time of the
        # sampled frame (not the publication time) so camera and ego
        # samples can be joined on the same simulator frame.
        sim_time = sample.get("sim_time", 0.0)
        stamp = _sim_stamp(sim_time) if sim_time > 0.0 else self.get_clock().now().to_msg()

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

        # Sim-time clock, one message per sampled frame alongside the ego
        # topics it timestamps (consumers with use_sim_time follow this).
        clock = Clock()
        clock.clock = stamp
        self.clock_pub.publish(clock)

        accel = AccelWithCovarianceStamped()
        accel.header.stamp = stamp
        accel.header.frame_id = "base_link"
        accel.accel.accel.linear.x = sample["ax"]
        accel.accel.accel.linear.y = sample["ay"]
        accel.accel.accel.linear.z = sample["az"]
        self.accel_pub.publish(accel)

        steer = SteeringReport()
        steer.stamp = stamp
        steer.steering_tire_angle = sample["steer"]
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
                # Stream watchdog: the world is ticking (find_actors got a
                # live vehicle) but no camera frame arrived for a while, so
                # the sensor session was purged server-side. Without this,
                # the CARLA client keeps requesting the dead stream id and
                # storms the server until its RPC starves. Re-listen to get
                # a fresh session; the retry is rate-limited by resetting
                # the timestamp even when the call fails.
                if (
                    self._camera is not None
                    and self._last_frame_at > 0.0
                    and time.monotonic() - self._last_frame_at > STREAM_WATCHDOG_SEC
                ):
                    try:
                        self._camera.stop()
                        self._camera.listen(self._on_camera)
                        self.get_logger().warn(
                            "camera stream stalled >%.1fs; re-listened"
                            % STREAM_WATCHDOG_SEC
                        )
                    except Exception as exc:
                        self.get_logger().warn("camera re-listen failed: %s" % exc)
                    self._last_frame_at = time.monotonic()
                # Read the world snapshot that the state below belongs to.
                # If the scenario advances the frame between the reads, the
                # sample would mix two frames; skip it and retry.
                snapshot = self._world.get_snapshot()
                frame = int(snapshot.frame)
                sim_time = float(snapshot.timestamp.elapsed_seconds)
                t = vehicle.get_transform()
                vel = vehicle.get_velocity()
                acc = vehicle.get_acceleration()
                if int(self._world.get_snapshot().frame) != frame:
                    time.sleep(0.005)
                    continue
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
                    "sim_time": sim_time,
                    "frame": frame,
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
                # Publish exactly once per sampled frame so every frame's
                # CARLA time appears on the ego topics and can be matched
                # to a camera frame.
                if frame != self._last_frame:
                    self._last_frame = frame
                    self._publish_sample(sample)
            except RuntimeError as exc:
                self._vehicle = None
                self._camera = None
                self._last_frame = None
                self.get_logger().warn("hero became unavailable: %s" % exc)
            time.sleep(0.01)

    def _measured_steer(self, vehicle) -> float:
        try:
            deg = vehicle.get_wheel_steer_angle(carla.VehicleWheelLocation.FL_Wheel)
            return -math.radians(float(deg))
        except Exception:
            return 0.0

    def _on_camera(self, image):
        try:
            self._last_frame_at = time.monotonic()
            arr = np.frombuffer(image.raw_data, dtype=np.uint8)
            arr = arr.reshape((image.height, image.width, 4))
            bgr = np.ascontiguousarray(arr[:, :, :3])
            # image.timestamp is the CARLA capture time (episode seconds);
            # image.frame is the CARLA frame number of this capture.
            with self._lock:
                self._ros_image = (
                    image.width,
                    image.height,
                    bgr,
                    float(image.timestamp),
                    int(image.frame),
                )
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
            width, height, bgr, capture_time, frame = item
            try:
                msg = Image()
                # Source identity: the CARLA capture time, not publication
                # time. VisionPilot preserves this stamp so its outputs can
                # be joined with the ego sample of the same frame.
                if capture_time > 0.0:
                    msg.header.stamp = _sim_stamp(capture_time)
                else:
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
                if frame % 50 == 0:
                    self.get_logger().debug(
                        "image frame=%d capture=%.3f" % (frame, capture_time)
                    )
            except Exception as exc:
                self.get_logger().error("image publish: %s" % exc)

    def _preview_loop(self):
        global _jpeg
        while not self._stop:
            with self._lock:
                frame = self._preview_frame
                self._preview_frame = None
            if frame is None:
                # 10 Hz preview: the tight 50 Hz loop starved the 1 Hz
                # evidence snapshots of CPU (observed ~0.1 Hz snaps).
                time.sleep(0.1)
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

    def _snap_loop(self):
        next_at = time.monotonic()
        while not self._stop:
            now = time.monotonic()
            if now < next_at:
                time.sleep(0.05)
                continue
            next_at = now + max(self._snap_period, 0.2)
            if not self._snap_dir:
                continue
            with self._lock:
                frame = self._preview_frame
            if frame is None:
                continue
            try:
                small = cv2.resize(frame, (640, 427))
                ok, buf = cv2.imencode(
                    ".jpg", small, [int(cv2.IMWRITE_JPEG_QUALITY), 60]
                )
                if not ok:
                    continue
                name = "snap_%d.jpg" % int(time.time() * 1000)
                with open(os.path.join(self._snap_dir, name), "wb") as f:
                    f.write(buf.tobytes())
                files = sorted(os.listdir(self._snap_dir))
                for old in files[: max(0, len(files) - self._snap_keep)]:
                    try:
                        os.remove(os.path.join(self._snap_dir, old))
                    except OSError:
                        pass
            except Exception as exc:
                self.get_logger().error("snap: %s" % exc)

    def _http(self):
        self._http_server = ThreadingHTTPServer(("0.0.0.0", 8090), _MjpegHandler)
        self._http_server.daemon_threads = True
        self._http_server.serve_forever()

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
