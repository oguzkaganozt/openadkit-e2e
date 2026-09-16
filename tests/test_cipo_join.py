import json
import sys
import tempfile
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
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "deploy" / "tools"))

from cipo_join import join_dumps  # noqa: E402
from scenario import along_components, along_speed, bumper_gap, sim_ns  # noqa: E402


class GeometryTests(unittest.TestCase):
    def test_along_is_forward_component(self):
        center, along = along_components(10.0, 0.0, 0.0, 1.0, 0.0, 0.0)
        self.assertAlmostEqual(center, 10.0)
        self.assertAlmostEqual(along, 10.0)

    def test_bumper_subtracts_half_lengths(self):
        self.assertAlmostEqual(bumper_gap(10.0, 2.4, 2.3), 5.3)

    def test_closing_lead_has_negative_rel_speed(self):
        hero = along_speed(10.0, 0.0, 0.0, 1.0, 0.0, 0.0)
        lead = along_speed(0.0, 0.0, 0.0, 1.0, 0.0, 0.0)
        self.assertAlmostEqual(lead - hero, -10.0)

    def test_sim_ns_rounds_tenths(self):
        self.assertEqual(sim_ns(24.1), 24100000000)


class JoinTests(unittest.TestCase):
    def test_joins_on_camera_sim_stamp(self):
        with tempfile.TemporaryDirectory() as tmp:
            dump = Path(tmp)
            (dump / "gt.jsonl").write_text(
                json.dumps(
                    {
                        "sim_t": 23.0,
                        "along": 15.9,
                        "bumper": 11.2,
                        "hero_v": 10.4,
                        "rel_v": -10.4,
                        "collision": 0,
                    }
                )
                + "\n"
                + json.dumps(
                    {
                        "sim_t": 24.1,
                        "along": 5.3,
                        "bumper": 0.5,
                        "hero_v": 10.6,
                        "rel_v": -10.6,
                        "collision": 1,
                    }
                )
                + "\n"
            )
            (dump / "vp.jsonl").write_text(
                json.dumps(
                    {
                        "cam_stamp_ns": 23000000000,
                        "fus": {"dist": 18.1, "vel": -1.5},
                        "latch": {
                            "in_fused": True,
                            "latched": False,
                            "last_dist": 18.1,
                            "out_dist": 18.1,
                        },
                    }
                )
                + "\n"
                + json.dumps(
                    {
                        "cam_stamp_ns": 24100000000,
                        "fus": {"dist": 13.5, "vel": -2.0},
                        "latch": {
                            "in_fused": True,
                            "latched": True,
                            "last_dist": 13.5,
                            "out_dist": 13.5,
                        },
                    }
                )
                + "\n"
            )
            joined, summary = join_dumps(dump)
            self.assertEqual(len(joined), 2)
            self.assertAlmostEqual(joined[0]["gt"]["along"], 15.9)
            self.assertEqual(summary["first_collision"]["sim_t"], 24.1)
            self.assertEqual(summary["first_latch"]["last_dist"], 13.5)
            self.assertAlmostEqual(summary["mae_dist_m"], ((18.1 - 15.9) + (13.5 - 5.3)) / 2)


if __name__ == "__main__":
    unittest.main()
