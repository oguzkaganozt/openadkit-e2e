import math
import unittest

from guard import (
    CLEAR_FRAMES,
    COMFORTABLE,
    EMERGENCY,
    FOLLOWER_TIMEOUT_MS,
    FRESH,
    HOLD,
    INIT,
    OWN_STOP_FRAME_ID,
    TRAJ_TIMEOUT_MS,
    EgoSnapshot,
    FollowerCmd,
    GuardPolicy,
    TrajSnapshot,
    is_own_stop,
)


def cmd(t, v=3.0, a=0.0, s=0.0):
    return FollowerCmd(stamp_ms=t, velocity=v, accel=a, steer=s)


def traj(t, n=5, v=3.0):
    return TrajSnapshot(stamp_ms=t, speeds=[v] * n, curvature=0.0,
                        all_zero=abs(v) < 0.05)


def ego(t, v=3.0):
    return EgoSnapshot(stamp_ms=t, speed=v)


def live(g, t, v=3.0):
    g.on_follower(cmd(t, v=v))
    g.on_traj(traj(t, v=v), t)
    g.on_odom(ego(t, v=v))


class GuardTests(unittest.TestCase):
    def test_startup_holds_until_valid(self):
        g = GuardPolicy()
        out = g.step(100.0)
        self.assertEqual(out.state, INIT)
        self.assertTrue(out.inhibit)
        self.assertIsNotNone(out.override_cmd)

    def test_fresh_passthrough(self):
        g = GuardPolicy()
        live(g, 1000.0)
        out = g.step(1010.0)
        self.assertEqual(out.state, FRESH)
        self.assertFalse(out.inhibit)
        self.assertIsNone(out.override_cmd)

    def test_stale_follower_goes_emergency(self):
        g = GuardPolicy()
        live(g, 1000.0)
        g.step(1010.0)
        out = g.step(1000.0 + FOLLOWER_TIMEOUT_MS + 10.0)
        self.assertEqual(out.state, EMERGENCY)
        self.assertIsNotNone(out.override_cmd)
        self.assertAlmostEqual(out.override_cmd.velocity, 0.0)
        self.assertLess(out.override_cmd.accel, 0.0)

    def _drive_comfortable(self, g):
        # Follower + odom fresh now, trajectory stale: comfortable entry.
        live(g, 1000.0)
        g.step(1010.0)
        step_t = g._traj_at + TRAJ_TIMEOUT_MS + 10.0
        g.on_follower(cmd(step_t))
        g.on_odom(ego(step_t))
        out = g.step(step_t)
        assert out.state == COMFORTABLE, out
        return step_t

    def test_stale_traj_goes_comfortable_while_follower_live(self):
        g = GuardPolicy()
        self._drive_comfortable(g)
        out = g.step(g._follower_at)
        self.assertEqual(out.state, COMFORTABLE)
        self.assertTrue(out.inhibit)
        self.assertTrue(out.publish_stop_traj)

    def test_follower_dead_skips_comfortable(self):
        g = GuardPolicy()
        live(g, 1000.0)
        g.step(1010.0)
        out = g.step(5000.0)  # everything stale
        self.assertEqual(out.state, EMERGENCY)

    def test_nan_follower_is_invalid(self):
        g = GuardPolicy()
        live(g, 1000.0)
        g.step(1010.0)
        g.on_follower(cmd(1020.0, v=float("nan")))
        out = g.step(1020.0)
        self.assertEqual(out.state, EMERGENCY)
        self.assertIn("invalid", out.reason)

    def test_babble_jump_goes_emergency(self):
        g = GuardPolicy()
        live(g, 1000.0, v=3.0)
        g.step(1010.0)
        g.on_follower(cmd(1020.0, v=3.0))
        g.on_follower(cmd(1030.0, v=25.0))
        g.on_odom(ego(1030.0))
        g.on_traj(traj(1030.0), 1030.0)
        out = g.step(1030.0)
        self.assertEqual(out.state, EMERGENCY)
        self.assertIn("babbling", out.reason)

    def test_envelope_violation_rejected(self):
        g = GuardPolicy()
        live(g, 1000.0)
        g.step(1010.0)
        # 10 m/s into a 0.05 rad/m curve = 5 m/s^2 lateral: outside.
        g.on_traj(TrajSnapshot(stamp_ms=1020.0, speeds=[10.0] * 5,
                               curvature=0.05), 1020.0)
        g.on_follower(cmd(1020.0, v=10.0, a=0.0))
        g.on_odom(ego(1020.0, v=10.0))
        out = g.step(1020.0)
        self.assertEqual(out.state, EMERGENCY)
        self.assertEqual(out.reason, "envelope-violation")

    def test_comfortable_recovers_when_traj_returns(self):
        g = GuardPolicy()
        base = self._drive_comfortable(g)
        for i in range(1, CLEAR_FRAMES + 1):
            t = base + i * 100.0
            g.on_follower(cmd(t))
            g.on_traj(traj(t), t)
            g.on_odom(ego(t))
            out = g.step(t)
        self.assertEqual(out.state, FRESH)
        self.assertFalse(out.inhibit)

    def test_emergency_holds_and_needs_re_enable(self):
        g = GuardPolicy()
        live(g, 1000.0, v=0.0)
        g.step(1010.0)
        g.on_follower(cmd(1020.0, v=float("nan")))
        g.on_odom(ego(1020.0, v=0.0))
        g.on_traj(traj(1020.0, v=0.0), 1020.0)
        out = g.step(1020.0)
        self.assertEqual(out.state, EMERGENCY)
        # ego stopped: next step latches HOLD
        g.on_follower(cmd(1030.0, v=0.0))
        g.on_odom(ego(1030.0, v=0.0))
        g.on_traj(traj(1030.0, v=0.0), 1030.0)
        out = g.step(1030.0)
        self.assertEqual(out.state, HOLD)
        # fault cleared but no re-enable: stays held
        live(g, 2000.0, v=0.0)
        out = g.step(2010.0)
        self.assertEqual(out.state, HOLD)
        # re-enable with the fault cleared is accepted
        out = g.step(2020.0, re_enable=True)
        self.assertEqual(out.state, FRESH)

    def test_re_enable_refused_while_fault_present(self):
        g = GuardPolicy()
        live(g, 1000.0, v=0.0)
        g.step(1010.0)
        out = g.step(5000.0)  # everything stale -> emergency...
        # feed ego stopped so it latches HOLD
        g.on_odom(ego(5001.0, v=0.0))
        out = g.step(5001.0)
        self.assertEqual(out.state, HOLD)
        out = g.step(5010.0, re_enable=True)
        self.assertEqual(out.state, HOLD)
        self.assertIn("refused", out.reason)

    def test_comfortable_steps_down_on_follower_death(self):
        g = GuardPolicy()
        base = self._drive_comfortable(g)
        out = g.step(base + FOLLOWER_TIMEOUT_MS + 100.0)
        self.assertEqual(out.state, EMERGENCY)

    def test_hard_stop_response_is_not_a_fault(self):
        # SI answers a sudden stop path with its own -5.0 emergency
        # decel: legitimate, must not trip the envelope (proven live).
        g = GuardPolicy()
        live(g, 1000.0, v=16.0)
        g.step(1010.0)
        g.on_follower(cmd(1020.0, v=16.0, a=-5.0))
        g.on_traj(traj(1020.0, v=16.0), 1020.0)
        g.on_odom(ego(1020.0, v=16.0))
        out = g.step(1020.0)
        self.assertEqual(out.state, FRESH)

    def test_runaway_throttle_still_trips(self):
        g = GuardPolicy()
        live(g, 1000.0, v=3.0)
        g.step(1010.0)
        g.on_follower(cmd(1020.0, v=3.0, a=5.0))
        g.on_traj(traj(1020.0), 1020.0)
        g.on_odom(ego(1020.0))
        out = g.step(1020.0)
        self.assertEqual(out.state, EMERGENCY)
        self.assertEqual(out.reason, "envelope-violation")

    def test_own_stop_loopback_is_ignored(self):
        self.assertTrue(is_own_stop(OWN_STOP_FRAME_ID))
        self.assertFalse(is_own_stop("map"))
        g = GuardPolicy()
        live(g, 1000.0)
        g.step(1010.0)
        # Only own stops arrive: trajectory must read stale, not fresh.
        step_t = 1000.0 + TRAJ_TIMEOUT_MS + 10.0
        g.on_traj(TrajSnapshot(stamp_ms=0.0, speeds=[0.0] * 5,
                               all_zero=True, own=True), step_t - 500.0)
        g.on_follower(cmd(step_t))
        g.on_odom(ego(step_t))
        out = g.step(step_t)
        self.assertEqual(out.state, COMFORTABLE)
        self.assertNotEqual(out.reason, "traj-recovered")

    def test_fresh_entry_resets_babble_baseline(self):
        # A STOPPED→DRIVE step across a hold boundary must not read as
        # babbling: entry into FRESH clears the jump baseline.
        g = GuardPolicy()
        live(g, 1000.0, v=0.0)
        g.step(1010.0)
        g.on_follower(cmd(1020.0, v=float("nan")))
        g.step(1020.0)
        self.assertEqual(g.state, EMERGENCY)
        g.on_follower(cmd(1030.0, v=0.0))
        g.on_odom(ego(1030.0, v=0.0))
        g.on_traj(traj(1030.0, v=0.0), 1030.0)
        g.step(1030.0)
        self.assertEqual(g.state, HOLD)
        live(g, 2000.0, v=0.0)
        out = g.step(2010.0, re_enable=True)
        self.assertEqual(out.state, FRESH)
        # First DRIVE command with a large steer step: no stale
        # baseline to compare against, so no babble trip.
        g.on_follower(cmd(2020.0, v=2.0, s=0.5))
        g.on_traj(traj(2020.0, v=2.0), 2020.0)
        g.on_odom(ego(2020.0, v=0.5))
        out = g.step(2020.0)
        self.assertEqual(out.state, FRESH)

    def test_brake_cmd_freezes_last_steer(self):
        g = GuardPolicy()
        live(g, 1000.0)
        g._last_follower = cmd(1000.0, s=0.2)
        b = g._brake_cmd(1010.0)
        self.assertAlmostEqual(b.steer, 0.2)
        self.assertAlmostEqual(b.velocity, 0.0)


if __name__ == "__main__":
    unittest.main()
