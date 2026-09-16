#!/usr/bin/env python3
"""Solve VisionPilot H.yaml from the live CARLA hero camera (ground-plane homography)."""

from __future__ import annotations

import argparse
import math

import carla
import cv2
import numpy as np


def camera_K(width: int, height: int, fov_deg: float) -> np.ndarray:
    f = width / (2.0 * math.tan(math.radians(fov_deg) / 2.0))
    return np.array(
        [[f, 0.0, width / 2.0], [0.0, f, height / 2.0], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def world_to_image(loc: carla.Location, cam, K: np.ndarray) -> tuple[float, float] | None:
    w2c = np.array(cam.get_transform().get_inverse_matrix())
    p = w2c @ np.array([loc.x, loc.y, loc.z, 1.0])
    x, y, z = p[1], -p[2], p[0]
    if z <= 0.1:
        return None
    u = K[0, 2] + K[0, 0] * x / z
    v = K[1, 2] + K[1, 1] * y / z
    return float(u), float(v)


def bumper_origin(hero) -> tuple[carla.Location, carla.Vector3D, carla.Vector3D]:
    tf = hero.get_transform()
    fwd = tf.get_forward_vector()
    left = carla.Vector3D(-tf.get_right_vector().x, -tf.get_right_vector().y, 0.0)
    loc = tf.location
    half = float(hero.bounding_box.extent.x)
    bumper = carla.Location(loc.x + fwd.x * half, loc.y + fwd.y * half, loc.z)
    wp = hero.get_world().get_map().get_waypoint(bumper)
    if wp is not None:
        bumper.z = wp.transform.location.z
    else:
        bumper.z = loc.z - float(hero.bounding_box.extent.z)
    return bumper, fwd, left


def ground_point(origin, fwd, left, x_fwd: float, y_left: float) -> carla.Location:
    return carla.Location(
        origin.x + fwd.x * x_fwd + left.x * y_left,
        origin.y + fwd.y * x_fwd + left.y * y_left,
        origin.z,
    )


def write_H(path: str, H: np.ndarray) -> None:
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
    fs.write("H", H)
    fs.release()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--out", default="deploy/config/H.yaml")
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1280)
    parser.add_argument("--fov", type=float, default=50.0)
    args = parser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(15.0)
    world = client.get_world()
    hero = None
    for actor in world.get_actors().filter("vehicle.*"):
        if actor.attributes.get("role_name") == "hero":
            hero = actor
            break
    if hero is None:
        raise SystemExit("hero not found")
    cam = None
    for actor in world.get_actors().filter("sensor.camera.rgb"):
        if actor.attributes.get("role_name") == "main_cam":
            cam = actor
            break
    if cam is None:
        raise SystemExit("main_cam not found")

    K = camera_K(args.width, args.height, args.fov)
    origin, fwd, left = bumper_origin(hero)
    world_xy = np.array(
        [[12.0, 2.0], [12.0, -2.0], [5.0, 2.0], [5.0, -2.0]], dtype=np.float64
    )
    img_pts = []
    for x, y in world_xy:
        loc = ground_point(origin, fwd, left, x, y)
        uv = world_to_image(loc, cam, K)
        if uv is None:
            raise SystemExit("ground point behind camera: %s %s" % (x, y))
        img_pts.append(uv)
        print("world X=%.1f Y=%+.1f -> u=%.1f v=%.1f" % (x, y, uv[0], uv[1]))
    img_pts = np.array(img_pts, dtype=np.float64)
    H, _ = cv2.findHomography(img_pts, world_xy)
    if H is None:
        raise SystemExit("findHomography failed")
    write_H(args.out, H)
    print("wrote", args.out)
    print(H)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
