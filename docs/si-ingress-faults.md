# VP ingress identity faults, latch persistence and VP restart

Measured on the GPU rig on 2026-09-25 with the final SI binary
(SHA-256 `a5481877…c8c82`). Each attempt ran in a freshly created CARLA world
by the wrapper scripts; failed or polluted attempts were discarded and never
retried in the same world.

## Reordered VP cycle (stale republish)

`deploy/tools/candidate_replay_probe.py` (run by
`deploy/tools/run-clean-vp-replay.sh`) waited for a NORMAL SI approval on
SI_CONTROL + VP, then republished the live VP candidate with a cycle five
behind the last approved one — same VP session, same topic, newer camera
stamp, so freshness alone could not legitimize it.

Observed (fresh world, attempt 1):

- SI latched `SI_STOP latched: vp trajectory candidate regression or invalid
  stamp (fault_id 1)` at `12:31:26.792`, 2.8 s after SI start;
- 0 NORMAL approvals while the latch was held, although the adapter kept
  publishing and 17 further valid candidates arrived after the injection;
- `mode=0` (SI_CONTROL) and `selected_source=0` (FOLLOWER) throughout — no
  mode or source change;
- one explicit `/control/safety_island/reenable` produced 3 NORMAL approvals
  on the same VP session (`3759794329`, cycles 56/58/59), then continued;
- the fault reached CARLA: `GATE STOP_APPLIED session=1485605219 seq=19
  mode=0 source=0 fault=1 frame=613 brake=0.400`.

Note: the tick that clears the latch still publishes one final explicit stop
with `fault_id=0` (`GATE STOP_APPLIED … seq=33 … fault=0`) before NORMAL
resumes on the next tick. This is a one-cycle (15 ms) artifact of
`callbackTimerControl` falling through to `publishSiStop()` in the same tick
that clears `supervision_.latched`; it is fail-safe and does not change
mode/source. It is documented rather than patched so the evidence binary
stays fixed; a follow-up can suppress that sample.

## VP restart (new producer session, same SI process)

`deploy/tools/run-clean-vp-restart.sh` started a fresh VP→SI_CONTROL world,
captured the healthy original session, recreated **only** the `visionpilot`
container (the `openadkit-e2e-si` container id was verified unchanged across
the restart), and observed both sides for 25 s:

- before: one session `1326393476`, 48 candidates, every NORMAL approval on
  that session, 0 stops;
- the VP outage exceeded the 1.0 s candidate watchdog, so SI latched
  `SI_STOP latched: trajectory` at `12:33:02.028` with the SI-stated source
  age `1.15 s`, and the actuator applied it
  (`GATE STOP_APPLIED session=1485691694 seq=77 mode=0 source=0 fault=1
  frame=850 brake=0.400`);
- after: the new session `3899277235` appeared on the candidate topic (the
  adapter republished it as `sess=3899277235 cyc=10`); the probe sent one
  explicit re-enable at `12:33:14.778` and SI resumed NORMAL with the new
  session (3 approvals, cycles 6–8);
- the original session was **never approved again** when the new one
  appeared (`old_approved_after_new: 0`), there was no unrequested resume
  (`unrequested_resume: 0`) and no mode/source change (`bad_selection: 0`).

## What these runs pin down

- A republished old candidate cannot refresh the selected source or clear a
  latch; only an explicit re-enable does, and only while all sources are
  fresh.
- A producer restart becomes a new session that SI accepts on its own, while
  the retired session stays rejected despite carrying later DDS samples.
- Mode and selected source are fixed at SI startup and are unaffected by
  either fault.

Raw logs: `/root/si-ingress-evidence-20260925/vp-replay-final/attempt-1/`
and `/root/si-ingress-evidence-20260925/vp-restart-final/attempt-1/` on the
rig host (SI, adapter, VP, actuator and scenario logs plus the binary hash).