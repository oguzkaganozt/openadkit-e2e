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
    sample_quadratic_path,
    stop_trajectory,
    serialized_size_bytes,
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

    def test_horizon_slows_along_path(self):
        path = sample_quadratic_path(0.0, 0.0, 0.0, x_max_m=20.0)
        horizon = [5.0 - 0.2 * i for i in range(20)]
        out = convert(path, Pose2D(0.0, 0.0, 0.0), speed_horizon=horizon)
        self.assertIsNotNone(out)
        assert out is not None
        self.assertGreater(out[0].longitudinal_velocity_mps, out[-1].longitudinal_velocity_mps)
        self.assertAlmostEqual(out[0].longitudinal_velocity_mps, 5.0, places=1)

    def test_configured_target_speed(self):
        self.assertAlmostEqual(target_speed_mps(3.0), 3.0)

    def test_yaw_roundtrip(self):
        for yaw in (0.0, 0.3, -1.2, math.pi / 2.0):
            x, y, z, w = quaternion_from_yaw(yaw)
            self.assertAlmostEqual(wrap_angle(yaw_from_quaternion(x, y, z, w)), wrap_angle(yaw))


if __name__ == "__main__":
    unittest.main()
