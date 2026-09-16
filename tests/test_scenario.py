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

from scenario import (  # noqa: E402
    along_components,
    bumper_gap,
    lead_stop_due,
    sim_ns,
    tm_speed_difference_percent,
)


class TrafficManagerSpeedTests(unittest.TestCase):
    def test_five_mps_on_ninety_kmh_limit(self):
        self.assertAlmostEqual(tm_speed_difference_percent(5.0, 90.0), 80.0)

    def test_at_limit_is_zero_percent(self):
        self.assertAlmostEqual(tm_speed_difference_percent(25.0, 90.0), 0.0)

    def test_zero_cruise_is_full_slowdown(self):
        self.assertAlmostEqual(tm_speed_difference_percent(0.0, 90.0), 100.0)

    def test_zero_limit_with_cruise_keeps_tm_at_limit(self):
        self.assertAlmostEqual(tm_speed_difference_percent(5.0, 0.0), 0.0)


class LeadStopTests(unittest.TestCase):
    def test_no_stop_configured_keeps_lead_moving(self):
        self.assertFalse(lead_stop_due(0.0, None))
        self.assertFalse(lead_stop_due(600.0, None))

    def test_stop_due_at_and_after_threshold(self):
        self.assertFalse(lead_stop_due(14.9, 15.0))
        self.assertTrue(lead_stop_due(15.0, 15.0))
        self.assertTrue(lead_stop_due(20.0, 15.0))


class GapGeometryTests(unittest.TestCase):
    def test_along_and_bumper(self):
        center, along = along_components(20.0, 0.0, 0.0, 1.0, 0.0, 0.0)
        self.assertAlmostEqual(center, 20.0)
        self.assertAlmostEqual(along, 20.0)
        self.assertAlmostEqual(bumper_gap(along, 2.5, 2.5), 15.0)

    def test_sim_ns(self):
        self.assertEqual(sim_ns(1.5), 1500000000)


if __name__ == "__main__":
    unittest.main()
