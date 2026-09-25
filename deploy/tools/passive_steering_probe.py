#!/usr/bin/env python3
"""Read-only CARLA steering trace for VP_CONTROL lateral diagnostics.

Run in a separate process during a *fresh* rig run. The script never ticks the
world or writes vehicle control. Compare its wall_ns/frame columns with VP,
SI, and actuator logs to distinguish command oscillation from wheel dynamics.
"""

import argparse
import csv
import math
import time

import carla


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", required=True, help="CSV output path")
    parser.add_argument("--seconds", type=float, default=45.0)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    args = parser.parse_args()

    client = carla.Client(args.host, args.port)
    client.set_timeout(3.0)
    world = client.get_world()
    # CARLA can return an empty actor registry immediately after a new client
    # connects; allow its replication to receive a world frame before failing.
    vehicle = None
    for _ in range(30):
        actors = world.get_actors()
        vehicle = next(
            (actor for actor in actors.filter("vehicle.*")
             if actor.attributes.get("role_name") == "hero"),
            None,
        )
        if vehicle is not None:
            break
        time.sleep(0.1)
    if vehicle is None:
        parser.error("no hero vehicle in world")
    maximum = math.radians(vehicle.get_physics_control().wheels[0].max_steer_angle)
    print("hero=%d max_steer=%.4f rad" % (vehicle.id, maximum), flush=True)

    fields = (
        "wall_ns", "frame", "sim_s", "x", "y", "yaw_deg", "speed_mps",
        "lane_off_m", "lane_curve_rad_per_m", "cmd_steer_norm",
        "cmd_tire_left_rad", "actual_fl_left_rad", "actual_fr_left_rad",
        "throttle", "brake",
    )
    cmap = world.get_map()
    deadline = time.monotonic() + args.seconds
    last_frame = -1
    with open(args.out, "w", newline="", encoding="utf-8") as trace:
        writer = csv.DictWriter(trace, fieldnames=fields)
        writer.writeheader()
        while time.monotonic() < deadline and vehicle.is_alive:
            snapshot = world.get_snapshot()
            if snapshot.frame == last_frame:
                time.sleep(0.01)
                continue
            last_frame = snapshot.frame
            transform = vehicle.get_transform()
            velocity = vehicle.get_velocity()
            control = vehicle.get_control()
            wp = cmap.get_waypoint(transform.location, project_to_road=True,
                                   lane_type=carla.LaneType.Driving)
            lane_off = float("nan")
            lane_curve = float("nan")
            if wp is not None:
                yaw = math.radians(wp.transform.rotation.yaw)
                dx = transform.location.x - wp.transform.location.x
                dy = transform.location.y - wp.transform.location.y
                lane_off = -math.sin(yaw) * dx + math.cos(yaw) * dy
                nxt = wp.next(2.0)
                if nxt:
                    nxt_yaw = math.radians(nxt[0].transform.rotation.yaw)
                    lane_curve = ((nxt_yaw - yaw + math.pi) % (2.0 * math.pi) - math.pi) / 2.0
            writer.writerow({
                "wall_ns": time.time_ns(),
                "frame": snapshot.frame,
                "sim_s": snapshot.timestamp.elapsed_seconds,
                "x": transform.location.x,
                "y": transform.location.y,
                "yaw_deg": transform.rotation.yaw,
                "speed_mps": math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2),
                "lane_off_m": lane_off,
                "lane_curve_rad_per_m": lane_curve,
                # CARLA's steer/FL-wheel sign is right-positive; the rig tire
                # angle and ROS DrivingCommand contract are left-positive.
                "cmd_steer_norm": control.steer,
                "cmd_tire_left_rad": -control.steer * maximum,
                "actual_fl_left_rad": -math.radians(vehicle.get_wheel_steer_angle(
                    carla.VehicleWheelLocation.FL_Wheel)),
                "actual_fr_left_rad": -math.radians(vehicle.get_wheel_steer_angle(
                    carla.VehicleWheelLocation.FR_Wheel)),
                "throttle": control.throttle,
                "brake": control.brake,
            })
            trace.flush()
    print("trace: %s (last frame=%d)" % (args.out, last_frame), flush=True)


if __name__ == "__main__":
    main()
