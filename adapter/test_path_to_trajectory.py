import math
import unittest

from path_to_trajectory import (
    DEFAULT_FRAME_ID,
    DEFAULT_V_MIN_MPS,
    EXTENT_CAP_M,
    SERIALIZED_BYTE_BUDGET,
    TARGET_POINT_COUNT,
    Pose2D,
    convert,
    cycle_reason,
    horizon_arc_lengths,
    reference_reason,
    sample_quadratic_path,
    sample_spatial,
    serialized_size_bytes,
    stamp_key,
    target_speed_mps,
    transform_to_odom,
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
    def test_empty_path_is_rejected(self):
        # No path means no usable reference: reject (None), never a
        # synthesized stop — the adapter must not refresh SI's input.
        out = convert([], Pose2D(10.0, 20.0, 0.0))
        self.assertIsNone(out)

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

    def test_no_horizon_is_rejected(self):
        path = sample_quadratic_path(0.0, 0.0, 0.0, x_max_m=10.0)
        out = convert(path, Pose2D(0.0, 0.0, 0.0))
        self.assertIsNone(out)

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


class ReferencePolicyTests(unittest.TestCase):
    # Target fault policy: a rejected reference stays silent; SI owns the
    # stop. The adapter only accepts a valid reference whose source stamp
    # matches an ego sample exactly (same CARLA world frame).

    def test_usable_reference_is_accepted(self):
        self.assertEqual(
            reference_reason(
                valid=True,
                path_valid=True,
                has_source_stamp=True,
                horizon_len=20,
                horizon_dt_s=0.05,
                x_max_m=30.0,
            ),
            "",
        )

    def test_invalid_flags_are_rejected(self):
        base = dict(
            valid=True,
            path_valid=True,
            has_source_stamp=True,
            horizon_len=20,
            horizon_dt_s=0.05,
            x_max_m=30.0,
        )
        cases = {
            "invalid-reference": dict(valid=False),
            "no-source-stamp": dict(has_source_stamp=False),
            "no-path": dict(path_valid=False),
            "no-horizon": dict(horizon_len=0),
        }
        for want, patch in cases.items():
            args = dict(base)
            args.update(patch)
            self.assertEqual(reference_reason(**args), want)

    def test_zero_x_max_is_no_path(self):
        self.assertEqual(
            reference_reason(
                valid=True,
                path_valid=True,
                has_source_stamp=True,
                horizon_len=20,
                horizon_dt_s=0.05,
                x_max_m=0.0,
            ),
            "no-path",
        )

    def test_degenerate_horizon_dt_is_rejected(self):
        self.assertEqual(
            reference_reason(
                valid=True,
                path_valid=True,
                has_source_stamp=True,
                horizon_len=20,
                horizon_dt_s=0.0,
                x_max_m=30.0,
            ),
            "no-horizon",
        )

    def test_cycle_accepts_first_and_new_session(self):
        self.assertEqual(cycle_reason(7, 0, -1, -1), "")
        self.assertEqual(cycle_reason(8, 0, 7, 150), "")

    def test_cycle_rejects_duplicate_and_regression(self):
        self.assertEqual(cycle_reason(7, 150, 7, 150), "cycle-not-increasing")
        self.assertEqual(cycle_reason(7, 149, 7, 150), "cycle-not-increasing")

    def test_stamp_key_is_exact(self):
        self.assertEqual(stamp_key(12, 500000000), (12, 500000000))
        self.assertNotEqual(stamp_key(12, 0), stamp_key(12, 1))

    def test_zero_budget_returns_none(self):
        # convert cannot fit a point into a zero byte budget; the caller
        # rejects instead of publishing anything.
        path = sample_quadratic_path(0.0, 0.0, 0.0, x_max_m=20.0)
        self.assertIsNone(
            convert(
                path,
                Pose2D(0.0, 0.0, 0.0),
                speed_horizon=[3.0] * 20,
                byte_budget=0,
            )
        )

    def test_cruise_override_ignores_horizon(self):
        # Library path used by the historical A/B baseline: constant speed
        # on the VP path shape, no horizon used.
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
