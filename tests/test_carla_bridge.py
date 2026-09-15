import sys
import types
import unittest
from pathlib import Path


def _stub(name):
    if name in sys.modules:
        return sys.modules[name]
    mod = types.ModuleType(name)
    def _getattr(attr):
        full = f"{name}.{attr}"
        val = _stub(full) if attr[:1].islower() else type(attr, (), {})
        setattr(mod, attr, val)
        return val
    mod.__getattr__ = _getattr
    sys.modules[name] = mod
    return mod


for _name in (
    "carla",
    "cv2",
    "numpy",
    "rclpy",
    "rclpy.executors",
    "rclpy.node",
    "rclpy.qos",
    "autoware_control_msgs",
    "autoware_control_msgs.msg",
    "autoware_vehicle_msgs",
    "autoware_vehicle_msgs.msg",
    "geometry_msgs",
    "geometry_msgs.msg",
    "nav_msgs",
    "nav_msgs.msg",
    "sensor_msgs",
    "sensor_msgs.msg",
    "std_msgs",
    "std_msgs.msg",
):
    _stub(_name)

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "deploy" / "nodes"))

from carla_bridge import (  # noqa: E402
    MAX_THROTTLE,
    STOP_SPEED_MPS,
    carla_longitudinal,
    carla_steer,
)


class LongitudinalTests(unittest.TestCase):
    def test_stop_command_brakes(self):
        throttle, brake = carla_longitudinal(0.0, -1.0, 3.0)
        self.assertEqual(throttle, 0.0)
        self.assertAlmostEqual(brake, 0.4)

    def test_stop_at_rest_still_brakes(self):
        throttle, brake = carla_longitudinal(STOP_SPEED_MPS, 0.0, 0.0)
        self.assertEqual(throttle, 0.0)
        self.assertAlmostEqual(brake, 0.4)

    def test_explicit_decel_brakes_before_speed_error(self):
        throttle, brake = carla_longitudinal(8.0, -0.6, 7.9)
        self.assertEqual(throttle, 0.0)
        self.assertGreater(brake, 0.0)

    def test_speed_error_brakes(self):
        throttle, brake = carla_longitudinal(3.0, 0.0, 4.0)
        self.assertEqual(throttle, 0.0)
        self.assertGreater(brake, 0.0)

    def test_cruise_uses_throttle(self):
        throttle, brake = carla_longitudinal(5.0, 0.5, 4.5)
        self.assertEqual(brake, 0.0)
        self.assertGreater(throttle, 0.0)
        self.assertLessEqual(throttle, MAX_THROTTLE)

    def test_throttle_is_capped(self):
        throttle, brake = carla_longitudinal(40.0, 5.0, 0.0)
        self.assertEqual(brake, 0.0)
        self.assertAlmostEqual(throttle, MAX_THROTTLE)


class SteerTests(unittest.TestCase):
    def test_positive_left_maps_to_carla_right(self):
        self.assertAlmostEqual(carla_steer(0.5, 1.0), -0.5)

    def test_clamps_to_unit_range(self):
        self.assertAlmostEqual(carla_steer(2.0, 1.0), -1.0)
        self.assertAlmostEqual(carla_steer(-2.0, 1.0), 1.0)

    def test_zero_max_steer_does_not_divide_by_zero(self):
        self.assertAlmostEqual(carla_steer(0.5, 0.0), -0.5)


if __name__ == "__main__":
    unittest.main()
