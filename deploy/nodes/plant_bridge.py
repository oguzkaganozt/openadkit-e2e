#!/usr/bin/env python3
"""SI DDS ↔ CARLA RPC. VP steering_cmd is not used."""

import math
import threading
import time
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


class PlantBridge(Node):
    def __init__(self):
        super().__init__("plant_bridge")
        self.declare_parameter("carla_host", "127.0.0.1")
        self.declare_parameter("carla_port", 2000)
        self.declare_parameter("role_name", "hero")
        self.declare_parameter("camera_role", "main_cam")
        host = self.get_parameter("carla_host").value
        port = int(self.get_parameter("carla_port").value)
        self._role = self.get_parameter("role_name").value
        self._camera_role = self.get_parameter("camera_role").value

        self._client = carla.Client(host, port)
        self._client.set_timeout(5.0)
        self._world = None
        self._vehicle = None
        self._camera = None
        self._last_steer = 0.0
        self._control_count = 0
        self._http_server = None

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
        self.get_logger().info("plant_bridge: CARLA RPC %s:%s http://0.0.0.0:8090/" % (host, port))

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
                bp = self._world.get_blueprint_library().find("sensor.camera.rgb")
                bp.set_attribute("image_size_x", "1920")
                bp.set_attribute("image_size_y", "1280")
                bp.set_attribute("fov", "50")
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
        if not self._connect():
            return
        self._find_actors()
        vehicle = self._vehicle
        if vehicle is None or not vehicle.is_alive:
            return
        try:
            t = vehicle.get_transform()
            vel = vehicle.get_velocity()
            acc = vehicle.get_acceleration()
        except RuntimeError as exc:
            # config_carla destroys the old hero before spawning the next one.
            self._vehicle = None
            self._camera = None
            self.get_logger().warn("hero became unavailable: %s" % exc)
            return
        yaw = -math.radians(t.rotation.yaw)
        qx, qy, qz, qw = _quat_from_yaw(yaw)
        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "map"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = float(t.location.x)
        odom.pose.pose.position.y = float(-t.location.y)
        odom.pose.pose.position.z = float(t.location.z)
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)
        vel_x = float(vel.x)
        vel_y = float(-vel.y)
        acc_x = float(acc.x)
        acc_y = float(-acc.y)
        odom.twist.twist.linear.x = cos_y * vel_x + sin_y * vel_y
        odom.twist.twist.linear.y = -sin_y * vel_x + cos_y * vel_y
        odom.twist.twist.linear.z = float(vel.z)
        self.odom_pub.publish(odom)

        speed = math.hypot(vel.x, vel.y)
        self.speed_pub.publish(Float64(data=float(speed)))

        accel = AccelWithCovarianceStamped()
        accel.header.stamp = stamp
        accel.header.frame_id = "base_link"
        accel.accel.accel.linear.x = cos_y * acc_x + sin_y * acc_y
        accel.accel.accel.linear.y = -sin_y * acc_x + cos_y * acc_y
        accel.accel.accel.linear.z = float(acc.z)
        self.accel_pub.publish(accel)

        steer = SteeringReport()
        steer.stamp = stamp
        steer.steering_tire_angle = self._last_steer
        self.steer_pub.publish(steer)

    def _on_camera(self, image):
        try:
            arr = np.frombuffer(image.raw_data, dtype=np.uint8)
            arr = arr.reshape((image.height, image.width, 4))
            bgr = np.ascontiguousarray(arr[:, :, :3])
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "hero/main_cam"
            msg.height = image.height
            msg.width = image.width
            msg.encoding = "bgr8"
            msg.is_bigendian = 0
            msg.step = image.width * 3
            msg.data = bgr.tobytes()
            self.image_pub.publish(msg)
            small = cv2.resize(bgr, (640, 427))
            ok, buf = cv2.imencode(".jpg", small, [int(cv2.IMWRITE_JPEG_QUALITY), 45])
            if ok:
                with _jpeg_lock:
                    global _jpeg
                    _jpeg = buf.tobytes()
        except Exception as exc:
            self.get_logger().error("camera: %s" % exc)

    def _http(self):
        self._http_server = ThreadingHTTPServer(("0.0.0.0", 8090), _MjpegHandler)
        self._http_server.daemon_threads = True
        self._http_server.serve_forever()

    def _on_control(self, msg: Control):
        if self._vehicle is None or not self._vehicle.is_alive:
            return
        try:
            self._last_steer = float(msg.lateral.steering_tire_angle)
            physics = self._vehicle.get_physics_control()
            max_steer = (
                math.radians(physics.wheels[0].max_steer_angle)
                if physics.wheels
                else 1.0
            )
            if max_steer <= 1e-6:
                max_steer = 1.0
            ctrl = carla.VehicleControl()
            # Autoware uses positive-left; CARLA's normalized input is positive-right.
            ctrl.steer = max(-1.0, min(1.0, -self._last_steer / max_steer))
            acc = float(msg.longitudinal.acceleration)
            vel = float(msg.longitudinal.velocity)
            if vel <= 0.05 and acc <= 0.0:
                ctrl.throttle = 0.0
                ctrl.brake = 0.4
            elif acc >= 0.0:
                ctrl.throttle = min(1.0, acc / 3.0)
                ctrl.brake = 0.0
            else:
                ctrl.throttle = 0.0
                ctrl.brake = min(1.0, -acc / 4.0)
            ctrl.hand_brake = False
            ctrl.manual_gear_shift = False
            self._vehicle.apply_control(ctrl)
            self._control_count += 1
            if self._control_count == 1 or self._control_count % 100 == 0:
                transform = self._vehicle.get_transform()
                self.get_logger().info(
                    "applied control #%d: tire=%.3f carla=%.3f throttle=%.3f "
                    "brake=%.3f pose=(%.2f, %.2f, %.1fdeg)"
                    % (
                        self._control_count,
                        self._last_steer,
                        ctrl.steer,
                        ctrl.throttle,
                        ctrl.brake,
                        transform.location.x,
                        transform.location.y,
                        transform.rotation.yaw,
                    )
                )
        except RuntimeError as exc:
            self._vehicle = None
            self._camera = None
            self.get_logger().warn("control target became unavailable: %s" % exc)

    def destroy_node(self):
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
    node = PlantBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
