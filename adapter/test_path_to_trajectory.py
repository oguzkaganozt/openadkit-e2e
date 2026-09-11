import math
import unittest

from path_to_trajectory import (
    DEFAULT_FRAME_ID,
    DEFAULT_V_MIN_MPS,
    EXTENT_CAP_M,
    SERIALIZED_BYTE_BUDGET,
    STALE_INPUT_MS,
    STOP_WATCHDOG_SEC,
    TARGET_POINT_COUNT,
    Pose2D,
    convert,
    horizon_arc_lengths,
    ingress_reason,
    sample_quadratic_path,
    sample_spatial,
    stop_trajectory,
    serialized_size_bytes,
    target_speed_mps,
    transform_to_odom,
    watchdog_due,
    yaw_from_quaternion,
    quaternion_from_yaw,
    wrap_angle,
)


class TransformTests(unittest.TestCase):
    def test_identity_at_origin(self):
        out = transform_to_odom(Pose2D(5.0, 1.0, 0.1), Pose2D(0.0, 0.0, 0.0))
        self.assertAlmostEqual(out.x, 5.0)
        self.assertAlmostEqual(out.y, 1.0)
        self.assertAlmostEqual(out.yaw, 0.1)

    def test_translate(self):
        out = transform_to_odom(Pose2D(5.0, 1.0, 0.0), Pose2D(10.0, 20.0, 0.0))
        self.assertAlmostEqual(out.x, 15.0)
        self.assertAlmostEqual(out.y, 21.0)

    def test_rotate_facing_plus_y(self):
        ego = Pose2D(10.0, 20.0, math.pi / 2.0)
        out = transform_to_odom(Pose2D(5.0, 0.0, 0.0), ego)
        self.assertAlmostEqual(out.x, 10.0)
        self.assertAlmostEqual(out.y, 25.0)
        self.assertAlmostEqual(out.yaw, math.pi / 2.0)


class QuadraticPathTests(unittest.TestCase):
    def test_straight_center(self):
        points = sample_quadratic_path(0.0, 0.0, 0.0, x_max_m=4.0, spacing_m=1.0)
        self.assertEqual(len(points), 5)
        self.assertEqual(points[0], Pose2D(0.0, 0.0, 0.0))
        self.assertEqual(points[-1], Pose2D(4.0, 0.0, 0.0))

    def test_cte_offset_is_y_at_ego(self):
        points = sample_quadratic_path(0.0, 0.0, 0.5, x_max_m=2.0, spacing_m=1.0)
        self.assertAlmostEqual(points[0].y, 0.5)

    def test_invalid_extent(self):
        self.assertEqual(sample_quadratic_path(0.0, 0.0, 0.0, x_max_m=0.0), [])


class ConvertTests(unittest.TestCase):
    def test_empty_path_is_stop(self):
        ego = Pose2D(10.0, 20.0, 0.0)
        out = convert([], ego)
        self.assertIsNotNone(out)
        assert out is not None
        self.assertEqual(len(out), 3)
        self.assertEqual((out[0].x, out[0].y), (ego.x, ego.y))
        self.assertAlmostEqual(out[0].longitudinal_velocity_mps, 0.0)
        self.assertAlmostEqual(out[1].longitudinal_velocity_mps, 0.0)
        self.assertAlmostEqual(out[2].longitudinal_velocity_mps, 0.0)
        self.assertGreater(out[1].x, out[0].x)

    def test_stop_trajectory_has_three_points(self):
        out = stop_trajectory(Pose2D(1.0, 2.0, math.pi / 2.0))
        self.assertEqual(len(out), 3)
        self.assertAlmostEqual(out[0].longitudinal_velocity_mps, 0.0)

    def test_straight_path_in_map(self):
        path = sample_quadratic_path(0.0, 0.0, 0.0, x_max_m=40.0)
        ego = Pose2D(100.0, 200.0, 0.0)
        out = convert(path, ego, target_velocity_mps=3.0)
        self.assertIsNotNone(out)
        assert out is not None
        self.assertLessEqual(len(out), TARGET_POINT_COUNT)
        self.assertAlmostEqual(out[0].x, 100.0, places=5)
        self.assertAlmostEqual(out[0].y, 200.0, places=5)
        self.assertGreater(out[-1].x, out[0].x)
        self.assertLessEqual(out[-1].x - out[0].x, EXTENT_CAP_M + 1.0)
        for p in out:
            self.assertAlmostEqual(p.longitudinal_velocity_mps, 3.0)
        self.assertAlmostEqual(out[0].time_from_start_sec, 0.0)
        self.assertGreater(out[-1].time_from_start_sec, 0.0)

    def test_fits_si_byte_budget(self):
        path = sample_quadratic_path(0.0, 0.0, 0.0, x_max_m=60.0)
        out = convert(path, Pose2D(0.0, 0.0, 0.0), target_velocity_mps=3.0)
        self.assertIsNotNone(out)
        assert out is not None
        size = serialized_size_bytes(len(DEFAULT_FRAME_ID), len(out))
        self.assertLessEqual(size, SERIALIZED_BYTE_BUDGET)
        self.assertLessEqual(len(out), TARGET_POINT_COUNT)

    def test_stopped_ego_still_gets_min_speed(self):
        path = sample_quadratic_path(0.0, 0.0, 0.0, x_max_m=10.0)
        out = convert(path, Pose2D(0.0, 0.0, 0.0), target_velocity_mps=0.0)
        self.assertIsNotNone(out)
        assert out is not None
        self.assertAlmostEqual(out[0].longitudinal_velocity_mps, DEFAULT_V_MIN_MPS)

    def test_distant_path_starts_at_ego(self):
        path = [Pose2D(5.0, 0.0, 0.0), Pose2D(10.0, 0.0, 0.0)]
        ego = Pose2D(1.0, 2.0, 0.0)
        out = convert(path, ego, target_velocity_mps=3.0)
        self.assertIsNotNone(out)
        assert out is not None
        self.assertEqual((out[0].x, out[0].y), (ego.x, ego.y))

    def test_no_horizon_is_stop(self):
        path = sample_quadratic_path(0.0, 0.0, 0.0, x_max_m=10.0)
        out = convert(path, Pose2D(0.0, 0.0, 0.0))
        self.assertEqual(len(out), 3)
        self.assertAlmostEqual(out[0].longitudinal_velocity_mps, 0.0)

    def test_arc_lengths_integrate_trapezoid(self):
        self.assertEqual(
            horizon_arc_lengths([0.0, 1.0, 2.0], 0.05), [0.0, 0.025, 0.1]
        )

    def test_spatial_sample_holds_beyond_extent(self):
        v, t = sample_spatial([0.0, 0.5], [1.0, 2.0], 5.0, 2.0, 0.05)
        self.assertAlmostEqual(v, 2.0)
        self.assertGreater(t, 0.05)

    def test_launch_reads_ahead(self):
        # VP: 0 -> 1.425 m/s over 1 s at +1.5 m/s^2, all inside 0.7 m of
        # travel. Reading 1 m ahead lands past the schedule end, so the
        # first point holds the schedule end: a rising demand that breaks
        # static friction with no special case, bounded by VP intent.
        path = sample_quadratic_path(0.0, 0.0, 0.0, x_max_m=20.0)
        horizon = [1.5 * 0.05 * i for i in range(20)]
        out = convert(path, Pose2D(0.0, 0.0, 0.0), speed_horizon=horizon)
        self.assertIsNotNone(out)
        assert out is not None
        speeds = [p.longitudinal_velocity_mps for p in out]
        self.assertAlmostEqual(speeds[0], horizon[-1])
        self.assertAlmostEqual(speeds[-1], horizon[-1])
        self.assertLessEqual(max(speeds), max(horizon) + 1e-9)
        times = [p.time_from_start_sec for p in out]
        self.assertTrue(all(math.isfinite(t) for t in times))
        self.assertTrue(
            all(b >= a for a, b in zip(times, times[1:])),
            "schedule time must not run backwards",
        )

    def test_flat_zero_horizon_holds_stop(self):
        # Genuine stop schedule (no motion in 1 s): honest zeros, so the
        # SI STOPPED hold engages instead of a false launch.
        path = sample_quadratic_path(0.0, 0.0, 0.0, x_max_m=20.0)
        out = convert(
            path, Pose2D(0.0, 0.0, 0.0), speed_horizon=[0.0] * 20
        )
        assert out is not None
        for p in out:
            self.assertAlmostEqual(p.longitudinal_velocity_mps, 0.0)

    def test_braking_ramps_down(self):
        # VP braking 8 -> 5 m/s: 1 m ahead on the falling schedule reads
        # below current speed (decel demand, never above actual), no
        # zeros anywhere, feedforward negative on the ramp.
        path = sample_quadratic_path(0.0, 0.0, 0.0, x_max_m=20.0)
        horizon = [8.0 - 0.15 * i for i in range(20)]
        out = convert(
            path, Pose2D(0.0, 0.0, 0.0), speed_horizon=horizon, vp_accel=-3.0
        )
        assert out is not None
        speeds = [p.longitudinal_velocity_mps for p in out]
        self.assertLess(speeds[0], 8.0)
        self.assertGreater(speeds[0], horizon[-1])
        self.assertNotIn(0.0, speeds)
        self.assertLess(out[0].acceleration_mps2, -1.0)
        self.assertLessEqual(max(speeds), max(horizon) + 1e-9)

    def test_slow_stop_collapses_to_hold(self):
        # VP stopping 2.0 -> 0.0 inside 0.7 m: 1 m ahead is past the
        # schedule end, so the profile holds the (zero) end: an honest
        # full stop with no feedforward left anywhere.
        path = sample_quadratic_path(0.0, 0.0, 0.0, x_max_m=20.0)
        horizon = [max(0.0, 2.0 - 0.15 * i) for i in range(14)] + [0.0] * 6
        out = convert(
            path, Pose2D(0.0, 0.0, 0.0), speed_horizon=horizon, vp_accel=-3.0
        )
        assert out is not None
        for p in out:
            self.assertAlmostEqual(p.longitudinal_velocity_mps, 0.0)
            self.assertAlmostEqual(p.acceleration_mps2, 0.0)

    def test_cruise_horizon_is_flat(self):
        path = sample_quadratic_path(0.0, 0.0, 0.0, x_max_m=20.0)
        out = convert(
            path, Pose2D(0.0, 0.0, 0.0), speed_horizon=[5.0] * 20
        )
        assert out is not None
        for p in out:
            self.assertAlmostEqual(p.longitudinal_velocity_mps, 5.0)
            self.assertAlmostEqual(p.acceleration_mps2, 0.0)

    def test_creep_horizon_has_no_false_stop(self):
        # Small positive schedule (standstill creep) must not contain
        # exact zeros: only a true VP zero may read as a stop.
        path = sample_quadratic_path(0.0, 0.0, 0.0, x_max_m=20.0)
        horizon = [0.05 + 0.015 * i for i in range(20)]
        out = convert(path, Pose2D(0.0, 0.0, 0.0), speed_horizon=horizon)
        assert out is not None
        speeds = [p.longitudinal_velocity_mps for p in out]
        # 1 m ahead is past this short schedule: hold the end, no zeros,
        # no special case. (0.3 m/s may sit under the actuation deadband;
        # that stalls safe, while a floor launched into bumpers.)
        self.assertAlmostEqual(speeds[0], horizon[-1], places=2)
        self.assertNotIn(0.0, speeds)

    def test_configured_target_speed(self):
        self.assertAlmostEqual(target_speed_mps(3.0), 3.0)

    def test_yaw_roundtrip(self):
        for yaw in (0.0, 0.3, -1.2, math.pi / 2.0):
            x, y, z, w = quaternion_from_yaw(yaw)
            self.assertAlmostEqual(wrap_angle(yaw_from_quaternion(x, y, z, w)), wrap_angle(yaw))


class IngressTests(unittest.TestCase):
    # Phase 1 fault policy: late / dropped / restarted / bad-shape input
    # each maps to an explicit stop downstream, never silence.

    def test_fresh_input_is_trusted(self):
        self.assertEqual(ingress_reason(5000.0, 4950.0, 4960.0), "")

    def test_no_horizon_yet(self):
        self.assertEqual(ingress_reason(5000.0, 0.0, 4960.0), "no-horizon-yet")

    def test_late_horizon_is_rejected(self):
        now = 5000.0
        self.assertEqual(
            ingress_reason(now, now - STALE_INPUT_MS - 1.0, now - 50.0),
            "stale-horizon",
        )

    def test_boundary_horizon_is_trusted(self):
        now = 5000.0
        self.assertEqual(
            ingress_reason(now, now - STALE_INPUT_MS, now - 50.0), ""
        )

    def test_stale_odom_is_rejected(self):
        now = 5000.0
        self.assertEqual(
            ingress_reason(now, now - 50.0, now - STALE_INPUT_MS - 1.0),
            "stale-odom",
        )

    def test_watchdog_fires_on_dropped_path(self):
        self.assertTrue(watchdog_due(10.0, 5.0))
        self.assertTrue(watchdog_due(10.0, 0.0))

    def test_watchdog_quiet_on_fresh_path(self):
        self.assertFalse(watchdog_due(10.0, 10.0 - STOP_WATCHDOG_SEC))
        self.assertFalse(watchdog_due(10.0, 9.9))

    def test_restart_recovers_on_fresh_arrival(self):
        # After a dropout, the first fresh arrival clears both gates:
        # ingress trusts it and the watchdog goes quiet.
        now = 60.0
        self.assertEqual(ingress_reason(now * 1000.0, now * 1000.0 - 50.0, now * 1000.0 - 50.0), "")
        self.assertFalse(watchdog_due(now, now))

    def test_bad_shape_maps_to_stop(self):
        # Zero byte budget can never fit a point: convert returns None
        # and the caller must answer with an explicit stop trajectory.
        path = sample_quadratic_path(0.0, 0.0, 0.0, x_max_m=20.0)
        self.assertIsNone(
            convert(
                path,
                Pose2D(0.0, 0.0, 0.0),
                speed_horizon=[3.0] * 20,
                byte_budget=0,
            )
        )
        stop = stop_trajectory(Pose2D(0.0, 0.0, 0.0))
        self.assertEqual(len(stop), 3)
        for p in stop:
            self.assertAlmostEqual(p.longitudinal_velocity_mps, 0.0)

    def test_cruise_override_ignores_horizon(self):
        # A/B knob: constant speed on the VP path shape, no horizon used.
        path = sample_quadratic_path(0.0, 0.0, 0.0, x_max_m=20.0)
        out = convert(
            path, Pose2D(0.0, 0.0, 0.0), target_velocity_mps=3.0,
            speed_horizon=None,
        )
        assert out is not None
        for p in out:
            self.assertAlmostEqual(p.longitudinal_velocity_mps, 3.0)


if __name__ == "__main__":
    unittest.main()
