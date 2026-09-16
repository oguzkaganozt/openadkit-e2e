import random
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "deploy" / "tools"))

from cipo_replay_fusion import (  # noqa: E402
    CIPO_NOISE,
    D_MAX,
    CameraFusion,
    _homography_noise,
)


class NoiseTests(unittest.TestCase):
    def test_homography_noise_grows_with_range(self):
        near = _homography_noise(30.0)
        far = _homography_noise(150.0)
        self.assertAlmostEqual(near, CIPO_NOISE)
        self.assertGreater(far, near)


class PolicyTests(unittest.TestCase):
    def test_fork_resets_without_confirm(self):
        fus = CameraFusion("fork", random.Random(0))
        out = fus.update(True, 0.2, 40.0, None, False, 0.1)
        self.assertTrue(out["reset"])
        self.assertAlmostEqual(out["dist"], D_MAX)

    def test_upstream_uses_ad_when_autospeed_has_box(self):
        fork = CameraFusion("fork", random.Random(1))
        up = CameraFusion("upstream", random.Random(1))
        fork_out = fork.update(True, 0.2, 40.0, 42.0, False, 0.1)
        up_out = up.update(True, 0.2, 40.0, 42.0, False, 0.1)
        self.assertFalse(fork_out["reset"])
        self.assertFalse(up_out["reset"])
        self.assertLess(abs(up_out["dist"] - 40.0), abs(fork_out["dist"] - 40.0))

    def test_upstream_still_resets_with_no_box_and_low_flag(self):
        up = CameraFusion("upstream", random.Random(2))
        out = up.update(True, 0.2, 40.0, None, False, 0.1)
        self.assertTrue(out["reset"])


if __name__ == "__main__":
    unittest.main()
