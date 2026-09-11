#!/usr/bin/env python3
"""Own the CARLA scenario, simulation ticks, actors, and spectator.

Pure CARLA PythonAPI — no ROS. Needs only the `carla` wheel matching this python
(deploy/build.sh downloads and verifies it). The CARLA bridge publishes ego telemetry as
ROS 2 topics, so ROS never crosses the host/container boundary.

The spawn point comes from the rig JSON ("spawn_index"); SPAWN_INDEX env overrides.
"""

import argparse
import json
import logging
import math
import os
import random
import signal
import time

import carla


def _check_versions(client):
    client_ver = client.get_client_version()
    server_ver = client.get_server_version()
    if client_ver.split("-")[0] != server_ver.split("-")[0]:
        logging.warning(
            "CARLA PythonAPI %s != server %s — API calls may segfault; stage the matching "
            "wheel with deploy/build.sh",
            client_ver,
            server_ver,
        )
    else:
        logging.info("CARLA client/server version %s", server_ver)


def _setup_vehicle(world, config):
    logging.debug("Spawning vehicle: {}".format(config.get("type")))

    bp_library = world.get_blueprint_library()
    map_ = world.get_map()

    bp = bp_library.filter(config.get("type"))[0]
    bp.set_attribute("role_name", config.get("id"))
    bp.set_attribute("ros_name", config.get("id"))

    spawn_points = map_.get_spawn_points()
    for i in range(len(spawn_points)):
        waypt = map_.get_waypoint(spawn_points[i].location)
        logging.debug(
            "Spawn Point {}: road {} lane {} section {}".format(
                i, waypt.road_id, waypt.lane_id, waypt.section_id
            )
        )

    # Priority: SPAWN_INDEX env > "spawn_index" in the rig JSON > 0.
    default_idx = int(config.get("spawn_index", 0))
    idx = int(os.environ.get("SPAWN_INDEX", default_idx))
    if not 0 <= idx < len(spawn_points):
        raise IndexError(
            "SPAWN_INDEX {} out of range: map {} has {} spawn points (valid 0..{})".format(
                idx, map_.name, len(spawn_points), len(spawn_points) - 1
            )
        )
    logging.info(
        "map %s: using spawn index %d (rig default %d; override with SPAWN_INDEX)",
        map_.name,
        idx,
        default_idx,
    )
    spawn_pt = spawn_points[idx]
    waypt = map_.get_waypoint(
        spawn_pt.location, project_to_road=True, lane_type=carla.LaneType.Driving
    )
    if waypt is not None:
        snapped = waypt.transform
        snapped.location.z = max(snapped.location.z, spawn_pt.location.z) + 0.05
        logging.info(
            "snapped spawn %d to road %s lane %s yaw %.1f (was %.1f)",
            idx,
            waypt.road_id,
            waypt.lane_id,
            snapped.rotation.yaw,
            spawn_pt.rotation.yaw,
        )
        spawn_pt = snapped

    return world.spawn_actor(bp, spawn_pt, attach_to=None)


def _setup_sensors(world, vehicle, sensors_config):
    bp_library = world.get_blueprint_library()

    sensors = []
    for sensor in sensors_config:
        logging.debug("Spawning sensor: {}".format(sensor))

        bp = bp_library.filter(sensor.get("type"))[0]
        bp.set_attribute("ros_name", sensor.get("id"))
        bp.set_attribute("role_name", sensor.get("id"))
        for key, value in sensor.get("attributes", {}).items():
            bp.set_attribute(str(key), str(value))

        wp = carla.Transform(
            location=carla.Location(
                x=sensor["spawn_point"]["x"],
                y=-sensor["spawn_point"]["y"],
                z=sensor["spawn_point"]["z"],
            ),
            rotation=carla.Rotation(
                roll=sensor["spawn_point"]["roll"],
                pitch=-sensor["spawn_point"]["pitch"],
                yaw=-sensor["spawn_point"]["yaw"],
            ),
        )

        sensors.append(world.spawn_actor(bp, wp, attach_to=vehicle))

    return sensors


def _follow_vehicle(world, vehicle, spectator):
    vehicle_transform = vehicle.get_transform()
    location = vehicle_transform.location
    rotation = vehicle_transform.rotation

    # Compute offset behind the vehicle in its local frame
    offset_distance = 6.0  # meters behind the vehicle
    height = 2.5  # meters above

    yaw_rad = math.radians(rotation.yaw)

    dx = -offset_distance * math.cos(yaw_rad)
    dy = -offset_distance * math.sin(yaw_rad)

    offset_location = carla.Location(x=location.x + dx, y=location.y + dy, z=location.z + height)

    spectator.set_transform(carla.Transform(offset_location, rotation))


def _setup_npc_traffic(world, traffic_manager, config, hero_spawn_index):
    bp_library = world.get_blueprint_library()
    map_ = world.get_map()
    spawn_points = map_.get_spawn_points()

    npc_vehicles = []
    for npc in config.get("npc_vehicles", []):
        count = npc.get("count", 1)
        for _ in range(count):
            bp_filter = npc.get("type", "vehicle.*")
            candidates = bp_library.filter(bp_filter)
            if not candidates:
                logging.warning("No blueprint matches '%s', skipping", bp_filter)
                continue
            bp = random.choice(candidates)
            if bp.has_attribute("color"):
                color = random.choice(bp.get_attribute("color").recommended_values)
                bp.set_attribute("color", color)

            idx = npc.get("spawn_index")
            if idx is not None:
                if not 0 <= idx < len(spawn_points):
                    logging.warning("npc spawn_index %d out of range, skipping", idx)
                    continue
                spawn_pt = spawn_points[idx]
            else:
                # avoid the hero's spawn point and any already-used ones
                free = [p for i, p in enumerate(spawn_points) if i != hero_spawn_index]
                spawn_pt = random.choice(free)

            actor = world.try_spawn_actor(bp, spawn_pt)
            if actor is None:
                logging.warning("Failed to spawn NPC (spawn point likely occupied)")
                continue

            if npc.get("autopilot", True):
                actor.set_autopilot(True, traffic_manager.get_port())

            npc_vehicles.append(actor)

    logging.info("Spawned %d NPC vehicles", len(npc_vehicles))
    return npc_vehicles


def _walk_lane(wp, dist_m, use_next):
    left = float(dist_m)
    while left > 0.5:
        cand = wp.next(2.0) if use_next else wp.previous(2.0)
        if not cand:
            break
        wp = cand[0]
        left -= 2.0
    return wp


def _in_front(hero_tf, loc):
    fwd = hero_tf.get_forward_vector()
    return (loc.x - hero_tf.location.x) * fwd.x + (loc.y - hero_tf.location.y) * fwd.y


def _spawn_lead(world, hero, ahead_m, traffic_manager):
    map_ = world.get_map()
    hero_tf = hero.get_transform()
    wp0 = map_.get_waypoint(
        hero_tf.location, project_to_road=True, lane_type=carla.LaneType.Driving
    )
    if wp0 is None:
        logging.warning("lead spawn: no waypoint at hero")
        return None
    a = _walk_lane(wp0, ahead_m, True)
    b = _walk_lane(wp0, ahead_m, False)
    wp = a if _in_front(hero_tf, a.transform.location) >= _in_front(
        hero_tf, b.transform.location
    ) else b
    bp = world.get_blueprint_library().filter("vehicle.tesla.model3")[0]
    bp.set_attribute("role_name", "lead")
    tf = wp.transform
    tf.location.z += 0.15
    actor = world.try_spawn_actor(bp, tf)
    if actor is None:
        logging.warning("lead spawn failed at yaw %.1f", tf.rotation.yaw)
        return None
    actor.set_autopilot(True, traffic_manager.get_port())
    traffic_manager.auto_lane_change(actor, False)
    traffic_manager.vehicle_percentage_speed_difference(actor, 50)
    logging.info(
        "lead on-lane ~%.0fm ahead yaw=%.1f at (%.1f,%.1f) hero yaw=%.1f at (%.1f,%.1f) front=%.1f",
        ahead_m,
        tf.rotation.yaw,
        tf.location.x,
        tf.location.y,
        hero_tf.rotation.yaw,
        hero_tf.location.x,
        hero_tf.location.y,
        _in_front(hero_tf, tf.location),
    )
    return actor


def main(args):
    world = None
    vehicle = None
    sensors = []
    npc_vehicles = []
    original_settings = None
    traffic_manager = None
    signal.signal(signal.SIGTERM, signal.default_int_handler)

    try:
        client = carla.Client(args.host, args.port)
        client.set_timeout(60.0)
        _check_versions(client)

        client.load_world("Town04")

        world = client.get_world()

        # Synchronous mode: this client explicitly drives the sim clock via world.tick(),
        # so each step advances by exactly fixed_delta_seconds. This removes the
        # wall-clock jitter seen in async mode (sensor_tick is measured in sim-time,
        # so async render-time variance made camera frame spacing uneven even though
        # the average rate looked close to the sensor_tick target).
        original_settings = world.get_settings()
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)

        traffic_manager = client.get_trafficmanager()
        traffic_manager.set_synchronous_mode(True)

        with open(args.file) as f:
            config = json.load(f)

        vehicle = _setup_vehicle(world, config)
        world.tick()
        sensors = _setup_sensors(world, vehicle, config.get("sensors", []))

        # Ground-truth instrumentation: collision events + hero/lead gap.
        # Lets us score VP distance estimates and contact time without
        # guessing from the camera.
        sim_t = [0.0]
        collisions = []
        col_bp = world.get_blueprint_library().find("sensor.other.collision")
        col_sensor = world.spawn_actor(col_bp, carla.Transform(), attach_to=vehicle)
        sensors.append(col_sensor)
        col_sensor.listen(
            lambda event: collisions.append(
                (
                    sim_t[0],
                    event.other_actor.type_id
                    if event.other_actor is not None
                    else "unknown",
                    event.normal_impulse.x,
                    event.normal_impulse.y,
                    event.normal_impulse.z,
                )
            )
        )

        # Spawn additional vehicles, avoiding the hero's (possibly env-overridden) spawn.
        hero_idx = int(os.environ.get("SPAWN_INDEX", config.get("spawn_index", 0)))
        lead_cfg = config.get("lead_vehicle") or {}
        lead = None
        lead_t = 0.0
        if lead_cfg.get("enabled"):
            lead = _spawn_lead(
                world, vehicle, lead_cfg.get("ahead_m", 30.0), traffic_manager
            )
            if lead is not None:
                npc_vehicles.append(lead)
        else:
            npc_vehicles = _setup_npc_traffic(world, traffic_manager, config, hero_idx)

        if args.autopilot:
            vehicle.set_autopilot(True)

        spectator = world.get_spectator()

        logging.info("Running... (ego up; telemetry is published by the bridge container)")

        TARGET_HZ = 20.0
        TARGET_PERIOD = 1.0 / TARGET_HZ

        tick = 0
        while True:
            loop_start = time.time()

            if lead is not None and lead.is_alive:
                lead_t += settings.fixed_delta_seconds or 0.05
                cruise_s = float(lead_cfg.get("cruise_s", 15.0))
                if lead_t >= cruise_s:
                    if abs(lead_t - cruise_s) < 0.08:
                        logging.info("lead braking now (t=%.1fs)", lead_t)
                        lead.set_autopilot(False)
                    ctrl = carla.VehicleControl(throttle=0.0, brake=0.8)
                    lead.apply_control(ctrl)

            world.tick()
            _follow_vehicle(world, vehicle, spectator)
            tick += 1
            sim_t[0] = tick * (settings.fixed_delta_seconds or 0.05)

            while collisions:
                t, other, ix, iy, iz = collisions.pop(0)
                logging.warning(
                    "COLLISION t=%.1fs with %s impulse=(%.1f,%.1f,%.1f)",
                    t,
                    other,
                    ix,
                    iy,
                    iz,
                )
            if lead is not None and lead.is_alive and tick % 20 == 0:
                hl = vehicle.get_location()
                ll = lead.get_location()
                hv = vehicle.get_velocity()
                lv = lead.get_velocity()
                logging.info(
                    "gap t=%.1fs center=%.1f hero_v=%.2f lead_v=%.2f",
                    sim_t[0],
                    math.hypot(hl.x - ll.x, hl.y - ll.y),
                    math.hypot(hv.x, hv.y),
                    math.hypot(lv.x, lv.y),
                )

            elapsed = time.time() - loop_start
            sleep_time = TARGET_PERIOD - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\nCancelled by user. Bye!")

    finally:
        # Block further KeyboardInterrupts during cleanup
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)

        try:
            if traffic_manager:
                traffic_manager.set_synchronous_mode(False)
            if original_settings:
                logging.info("Restoring original settings")
                world.apply_settings(original_settings)

            for sensor in sensors:
                if sensor.is_alive:
                    logging.debug("Destroying sensor: {}".format(sensor.type_id))
                sensor.destroy()

            for npc in npc_vehicles:
                if npc.is_alive:
                    logging.debug("Destroying NPC: {}".format(npc.type_id))
                npc.destroy()

            if vehicle:
                if vehicle.is_alive:
                    logging.debug("Destroying vehicle: {}".format(vehicle.type_id))
                vehicle.destroy()

        finally:
            # Re-enable KeyboardInterrupt handling
            signal.signal(signal.SIGINT, signal.default_int_handler)


if __name__ == "__main__":
    argparser = argparse.ArgumentParser(description="CARLA ego spawn")
    argparser.add_argument(
        "--host",
        metavar="H",
        default="localhost",
        help="IP of the host CARLA Simulator (default: localhost)",
    )
    argparser.add_argument(
        "--port",
        metavar="P",
        default=2000,
        type=int,
        help="TCP port of CARLA Simulator (default: 2000)",
    )
    argparser.add_argument("-f", "--file", default="", required=True, help="File to be executed")
    argparser.add_argument(
        "-v", "--verbose", action="store_true", dest="debug", help="print debug information"
    )
    argparser.add_argument(
        "-a",
        "--autopilot",
        action="store_true",
        dest="autopilot",
        help="turn on autopilot for the vehicle",
    )

    args = argparser.parse_args()

    log_level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(format="%(levelname)s: %(message)s", level=log_level)

    logging.info("Listening to server %s:%s", args.host, args.port)

    main(args)
