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


_stub("carla")

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "deploy" / "nodes"))

from scenario import tm_speed_difference_percent  # noqa: E402


class TrafficManagerSpeedTests(unittest.TestCase):
    def test_five_mps_on_ninety_kmh_limit(self):
        self.assertAlmostEqual(tm_speed_difference_percent(5.0, 90.0), 80.0)

    def test_at_limit_is_zero_percent(self):
        self.assertAlmostEqual(tm_speed_difference_percent(25.0, 90.0), 0.0)

    def test_zero_cruise_is_full_slowdown(self):
        self.assertAlmostEqual(tm_speed_difference_percent(0.0, 90.0), 100.0)

    def test_zero_limit_with_cruise_keeps_tm_at_limit(self):
        self.assertAlmostEqual(tm_speed_difference_percent(5.0, 0.0), 0.0)


if __name__ == "__main__":
    unittest.main()
