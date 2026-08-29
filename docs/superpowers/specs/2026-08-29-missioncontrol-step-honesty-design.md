# MissionControl Step Honesty — Design

**Date:** 2026-08-29
**Status:** Accepted 2026-08-29
**Goal:** Make `DroneControlImpl::step()` do exactly one scan per mission step, matching the
documented movement → scan → fuse contract, and honor a command that carries both a movement and a
scan instead of silently discarding the movement.

This is project **B** of four. See `docs/superpowers/specs/2026-08-29-mapping-algorithm-roadmap.md`
for the full sequence, dependencies, and the two decisions (honest-step-accounting scope; beat-ex2
bar) that constrain this project.

---

## Problem

`DroneControlImpl::step()` batches up to 16 scans into a single mission step
(`MissionControl/src/DroneControlImpl.cpp:187-214`, `kMaxScansPerStep = 16`), re-invoking
`nextStep` inside the loop on every scan. Two consequences:

1. **Step accounting is dishonest relative to the contract.** A foreign MissionControl does exactly
   one scan per step — that's the VAR-02 fixture's behavior
   (`Simulator/tests/fixtures/foreign_hits_only_mission_control_plugin.cpp:77-134`) and the contract
   `docs/superpowers/specs/2026-08-28-independent-component-variants-design.md:109-113` documents for
   it. Our batching means the same algorithm costs ~2 steps per observation stop under our MC and
   ~26 under a foreign one — Known Issues #20 measured this directly: 209 vs 42 steps on
   `small_simulation_room`, ~5×.

2. **A command carrying both `movement` and `scan_orientation` silently loses the movement.** The
   scan loop runs first — because it checks `command.scan_orientation.has_value()` before anything
   else looks at `command.movement` — and each loop iteration overwrites `command` with the result
   of a fresh `nextStep` call. The movement block after the loop only ever sees the *last* `nextStep`
   result inside that batch (`DroneControlImpl.cpp:216-248`). This is latent today because our
   algorithm never emits both (`Algorithm/tests/test_mapping_algorithm.cpp:355-370`), but project D's
   next-best-view rewrite is expected to emit both routinely — that's the entire benefit of
   `docs/mapping-algorithm-analysis.md`'s alternative #1 ("Emit movement and scan in the same
   command").

Separately, `.cursor/rules/frozen-interfaces.mdc:66-67` documents the drone-control step order as
movement → scan → fuse. The current code does the opposite — the full scan batch runs before
movement is ever considered. This spec resolves that mismatch by making the code match the
documented order, not by changing the rule.

---

## What does not change

- Decision 1 from the roadmap: our MissionControl keeps carving `Empty` along beams
  (`ScanResultToVoxels::applyToMap`, `supplementGridAlignedFusion`) and keeps passing non-null
  `latest_scan_`. B only changes *how many scans per step* and *whether movement survives alongside
  a scan* — not what the scan does once taken.
- The terminal-status short-circuit: if `nextStep` returns `Finished` or
  `FinishedWithUnmappableVoxels`, `step()` returns `Completed` immediately, with no increment to
  `step_index_`, no movement, no scan. Unchanged.
- A hard (non-recoverable) movement failure returns `Error` immediately, with no scan and no
  increment. Unchanged.
- `isRecoverableMovementFailure`'s `blocked`/`boundary` substring match
  (`DroneControlImpl.cpp:93-96`). Unchanged — that's project C's concern (the clearance-check bug
  that makes this substring match load-bearing), not B's.
- Movement-limit validation (`movementWithinLimits`) and its `Error` return on violation. Unchanged.

---

## Design

### New `step()` flow

`nextStep` is called **exactly once** per `step()` call — never twice, never in a loop. This single
change is what removes the batching loop, the `kMaxScansPerStep` constant, and the co-emission bug
simultaneously: movement and scan both come from the same `command`, so nothing gets overwritten.

```text
DroneControlImpl::step():
  state ← current state; clear drone footprint at state.position
  command ← mapping_algorithm_.nextStep(state, latest_scan_ptr)

  if command.status is Finished or FinishedWithUnmappableVoxels:
      return Completed                              // no increment, no movement, no scan

  if command.movement has a value:
      if not withinLimits(command.movement):
          return Error("Movement command exceeds drone limits.")   // no increment, no scan
      result ← executeMovement(command.movement)
      if result failed:
          if isRecoverableMovementFailure(result.message):
              // stationary but functional — fall through to the scan step below
          else:
              return Error(result.message)          // no increment, no scan

  if command.scan_orientation has a value:
      latest_scan_ ← lidar_sensor_.scan(*command.scan_orientation)
      has_latest_scan_ ← true
      ScanResultToVoxels::applyToMap(output_map_, gps_.position(), gps_.heading(), latest_scan_, lidar_)
      supplementGridAlignedFusion(output_map_, gps_.position(), gps_.heading(), latest_scan_, lidar_.z_max)
      clear drone footprint at gps_.position()        // post-movement position/heading

  ++step_index_
  return Continue
```

The scan, when taken, always uses `gps_.position()`/`gps_.heading()` fetched fresh after the movement
attempt (successful, recoverably-failed-in-place, or absent) — never the pre-movement `state`. This
matches the documented order (movement, then scan from wherever the drone actually ended up) and
matches what the VAR-02 fixture already does.

### Why the recoverable-failure path still scans

A recoverable movement failure means the drone didn't move but is otherwise functional. Skipping the
scan in that case would waste a scan the algorithm explicitly asked for, for no contract reason —
nothing about a blocked/boundary bump makes the drone's sensor stop working. The hard-failure path
still short-circuits before any scan, since a hard error means `step()` is reporting a run-ending
condition upstream.

### Why removing the loop (not capping it at 1) resolves the hang risk

Known Issues #21 root-caused a hang in `while (command.scan_orientation.has_value() && ... )` when an
adversarial algorithm always returns `Working` + a scan orientation — the loop never terminated on
its own regardless of the cap, and `kMaxScansPerStep = 16` was the fix (a bound, not a removal of the
unbounded-iteration structure). With B, there is no loop inside `step()` to hang: `nextStep` is called
once, `step()` returns, and the *outer* `MissionControlImpl::runMission()` loop — already bounded by
`mission_config_.max_steps` — is what iterates. The hang class is structurally impossible rather than
capped.

---

## Test plan

### Re-verify before assuming no fallout

A prior survey found no existing gtest hard-codes `kMaxScansPerStep == 16` or asserts multi-scan
batching inside one `step()` call (`MissionControl/tests/test_drone_control.cpp:363`'s
`ExecutesScanThenContinues` asserts `scan_count_ == 1`, which still holds). Re-grep at implementation
time to confirm before relying on it — summaries age.

### New cases

| Test | Asserts |
|------|---------|
| Movement + scan in one command | Both `executeMovement` and `lidar.scan` are invoked from a single `step()` call carrying both fields; map is fused; `step_index_` increments once |
| Recoverable movement failure still scans | A blocked/boundary movement failure is followed by the scan executing and the map being fused before returning `Continue` |
| Hard movement failure skips the scan | A non-recoverable movement failure returns `Error` without any `lidar.scan` call |
| One scan per step, unconditionally | An algorithm fixture that always returns `Working` + a scan orientation triggers exactly one `lidar.scan()` call per `step()` call, across many consecutive calls — the direct replacement for the old batch-cap guarantee |
| Terminal short-circuit unchanged | `Finished`/`FinishedWithUnmappableVoxels` still returns `Completed` with no movement, no scan, no increment |

### Regression checks

- `Simulator/tests/manual/check_adversarial_plugins.sh` — `bad_scan` should remain safe and become
  faster/more clearly bounded, not riskier, since the hang class is now structurally impossible.
- `Simulator/tests/manual/check_foreign_mission_control.sh` (VAR-02 diagnostic) — re-run after B
  lands; Known Issues #20's "209 vs 42" numbers are expected to converge substantially, since both
  hosts now do one scan per step. Any remaining gap should come from Empty-carving / non-null-scan
  differences, not batching — document whatever gap remains rather than assuming it's zero.
- Full `ctest` suite (85/85 documented) — run to confirm no incidental breakage, not just the tests
  named above.
- Project A's harness (`ex2_comparable` snapshot vs a fresh `honest` sweep) — expected to show a
  regression on tight-budget missions (`inputs/mission/large_mission_room.yaml`, 500 steps). This is
  the anticipated, accepted fall described in the roadmap — record it, do not treat it as a bug to
  chase within B's scope.

---

## Documentation updates

| Doc | Change |
|-----|--------|
| `docs/HLD.md:251-253` | Remove "batched in one step" language; describe the single-scan-per-step flow |
| `docs/known-issues.md` #20 | Mark superseded: step counts converge once both hosts do one scan per step; note what gap (if any) remains post-remeasurement |
| `docs/known-issues.md` #21 | Add a note that the interim `kMaxScansPerStep = 16` fix was superseded by removing the loop entirely in B, and why that's a stronger fix (structural, not a bound) |
| `docs/mapping-algorithm-analysis.md:140-151` | Update the batching discussion to reflect it's resolved; keep the doc as a historical record of the finding, don't delete the section |
| `docs/superpowers/specs/2026-08-28-independent-component-variants-design.md:109-113` | Update the "our behavior" column for the batching row now that it matches the foreign variant |
| `.cursor/rules/frozen-interfaces.mdc:66-67` | No change — code now matches this rule; confirm during review rather than editing |

---

## Out of scope

- Any change to `Algorithm/`. The current algorithm never emits both `movement` and
  `scan_orientation` in one command, so the co-emission path is exercised only by B's new tests until
  project D lands.
- The clearance-check / `isSpherePassable` bug and the `blocked`/`boundary` substring match's
  robustness — project C.
- Any change to movement-limit validation or GPS resolution handling.
- Re-tuning the algorithm's own scan pattern or frontier policy in response to the new step cost —
  project D.

---

## Success criteria

1. `DroneControlImpl::step()` calls `nextStep` at most once per invocation.
2. A command with both `movement` and `scan_orientation` set executes both within one `step()` call.
3. `kMaxScansPerStep` and the scan-batching loop are removed, not capped.
4. New tests in the table above pass; full `ctest` suite remains green; `check_adversarial_plugins.sh`
   still passes.
5. Project A's harness shows the `honest` column, run post-B on the unmodified current algorithm,
   with the regression on tight-budget cells documented in `docs/benchmarks/`.
6. The listed docs are updated; `frozen-interfaces.mdc` requires no edit (confirmed, not assumed).

---

## Related docs

- `docs/superpowers/specs/2026-08-29-mapping-algorithm-roadmap.md` — full project sequence and
  the decisions this spec inherits
- `docs/superpowers/specs/2026-08-29-algorithm-benchmark-harness-design.md` — project A, used to
  measure this project's before/after
- `docs/mapping-algorithm-analysis.md` — the review that identified both findings this spec fixes
- `docs/known-issues.md` — #20, #21
- `docs/superpowers/specs/2026-08-28-independent-component-variants-design.md` — VAR-02 foreign MC
  contract this project aligns toward
