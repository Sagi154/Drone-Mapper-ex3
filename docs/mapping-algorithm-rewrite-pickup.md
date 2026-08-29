# Pickup — Mapping algorithm rewrite (A/B/C/D)

**Read this first** when continuing the “better algorithm” work. It is the standing handoff from
2026-08-29: what landed on branch `algorithm-benchmark-harness`, measured scores, and the ordered
queue for C → D. Do not restart from the deadline-driven “patch the current policy” framing in older
docs — quality is the constraint; the four-project roadmap is authoritative.

**Not this file:** assignment packaging / zip / Known Issues excel → still
`docs/assignment-compliance-pickup.md`. That track is separate from the algorithm rewrite.

---

## Branch / git

| Item | Value |
|------|--------|
| Branch | `algorithm-benchmark-harness` (from up-to-date `main` at start of work) |
| vs `main` | **14 commits ahead**, not pushed (as of handoff) |
| Tip (approx) | `docs: record post-B honest-column score regression` |

Working tree should be clean after the B commits. If dirty, stop and reconcile before starting C.

---

## Verdict

**Projects A and B are done.** We can measure per-cell score/steps, and our MissionControl is honest
(one scan per step, movement→scan→fuse, co-emitted movement+scan honored). The current algorithm
**does not** meet the beat-ex2 bar under that honest contract. Recovering coverage-per-step is the
job of **C then D** — not further MC tweaks.

Two decisions that still constrain everything below (do not re-litigate without the user):

1. **Optimization target = honest step accounting only** on our MC (carve Empty, pass `latest_scan`).
   Adversarial hits-only MC is a robustness check, not the primary objective.
2. **Success = beat ex2’s recorded 24-cell score bands** (same maps/scorer), not merely
   `mission_score >= 0`.

Full rationale and open questions for C/D live in:
`docs/superpowers/specs/2026-08-29-mapping-algorithm-roadmap.md`.

---

## Done

### Project A — benchmark harness

| Artifact | Path |
|----------|------|
| Spec | `docs/superpowers/specs/2026-08-29-algorithm-benchmark-harness-design.md` |
| Plan | `docs/superpowers/plans/2026-08-29-algorithm-benchmark-harness.md` |
| Tool | `scripts/benchmark/` (`run_benchmark.py`, parser, summarize, pytest) |
| Ex2 bands | `docs/benchmarks/ex2-reference.csv` |
| Pre-B baseline | `docs/benchmarks/2026-08-29-pre_b_baseline.{csv,md}` (column `ex2_comparable` + `adversarial`) |

Also: `docs/mapping-algorithm-analysis.md` (policy review that started this work).

### Project B — MissionControl honesty

| Artifact | Path |
|----------|------|
| Spec | `docs/superpowers/specs/2026-08-29-missioncontrol-step-honesty-design.md` |
| Plan | `docs/superpowers/plans/2026-08-29-missioncontrol-step-honesty.md` |
| Code | `MissionControl/src/DroneControlImpl.cpp` — no scan-batch loop; `nextStep` once; move then scan |
| Tests | `MissionControl/tests/test_drone_control.cpp` — co-emission, recoverable+scan, hard-fail skips scan, one scan/step |
| Docs | HLD, known-issues #20/#21, mapping-algorithm-analysis portability section, VAR-02 table |
| Post-B baseline | `docs/benchmarks/2026-08-29-post_b_honest.{csv,md}` |

---

## Measured scores (primary numbers)

### Pre-B (batching MC) — `ex2_comparable`

- Score sum **1371.3**, steps **4946**, 0× `MAX_STEPS`, 24 scored.
- vs ex2 bands: **inside** house_lower, large_room; **below** house_full (~10), large_out (~36), small_out (~37), small_room (~65).

### Post-B (honest MC) — `honest` ← **optimize against this**

- Score sum **1335.4**, steps **17693** (~3.6× pre-B), **8× `MAX_STEPS`**, 0 errors.
- Same band shape vs ex2 (still only inside on house_lower + large_room).

### Adversarial (hits-only foreign MC)

- Pre-B column in `pre_b_baseline`: score sum ~536, steps ~6982, 4× `MAX_STEPS`. Robustness only.

**How to re-measure:** see `scripts/benchmark/README.md`. Docker image may need
`apt-get install -y python3-venv python3-pip` once per container. Full 24-cell sweep ~2–5+ minutes
per column at `num_threads=8`.

---

## Left to do (in order)

### 1. Project C — sensor model + clearance / belief (specced, next: plan + implement)

**Status:** specced 2026-08-29. Spec:
`docs/superpowers/specs/2026-08-29-sensor-model-clearance-belief-design.md`.

1. **Fix `isSpherePassable` no-op** (safety — foreign hard `Error` → run score `-1`). Confirmed the bug
   directly: `ceil(radius_cm/step_cm) = 1` for both shipped drone radii vs 10 cm grid, and the probe
   loop's own range gate then skips every non-centre offset — the ~55-line sweep degrades to one
   `atVoxel` call. Fix: nearest-point-in-box-to-sphere test on the fixed 3×3×3 neighbourhood, not a
   bigger `Unmapped`-traversal policy change.
2. **Derive scan directions from lidar** (`d` / `z_min` / `fov_circles`); gain-gate scans against
   `output_map_` (no new belief-map structure — decision 1 keeps `output_map_` as ground truth).
3. **Deferred to D, only if needed:** own belief from `latest_scan`, degrade when scans are null
   (competition foreign MC) — D's viewpoint-scoring defines what it actually needs.
4. Shared beam math → `UserCommon/` (currently empty of geometry code), lifted from
   `MissionControl/src/BeamMath.hpp` / `ScanResultToVoxels`, scoped to what gain-gating needs.

**Next session action:** write C's implementation plan (writing-plans skill) → execute via SDD →
re-run harness `honest` column → commit `docs/benchmarks/*` deltas.

### 2. Project D — exploration policy (NBV)

**Status:** not specced. Depends on C. Receding-horizon next-best-view, read `max_steps`, any-angle
smoothing, emit move+scan, retire blacklists / stale `explore_dist_cache` / mid-search Dijkstra hack,
`mp-units` geometry. Deterministic sampling (fixed seed) required for harness.

### 3. After C/D land

- Commit new `docs/benchmarks/*` CSVs; update roadmap tables.
- Confirm adversarial column still completes without illegal-move disasters.
- Open PR(s) for `algorithm-benchmark-harness` (consider splitting docs/A vs B vs C/D if review size
  hurts) — **do not push to `main`**.
- Separate track: zip packaging / Known Issues excel (`assignment-compliance-pickup.md`).

---

## Next session — concrete first steps

1. Read this file + `docs/superpowers/specs/2026-08-29-sensor-model-clearance-belief-design.md` (C's
   spec, done) + `docs/mapping-algorithm-analysis.md` clearance / scan-geometry findings.
2. Confirm on `algorithm-benchmark-harness`, `git status` clean, optionally `git log main..HEAD --oneline`.
3. Write **C's implementation plan** and execute via SDD (do not jump to D).
4. After C lands, re-benchmark `honest` before starting D’s full policy rewrite.

---

## Key constraints for implementers

- Human approval before every `git commit` (`.cursor/rules/git-workflow.mdc`).
- Never edit frozen `common/` / published headers.
- Algorithm must stay independently loadable (no cross-`.so` deps on our MC symbols).
- Shared beam math → `UserCommon/` sources compiled into each plugin, not duplicated (e10).

---

## Related docs

| Doc | Role |
|-----|------|
| `docs/superpowers/specs/2026-08-29-mapping-algorithm-roadmap.md` | Living A/B/C/D index + C/D considerations |
| `docs/superpowers/specs/2026-08-29-sensor-model-clearance-belief-design.md` | Project C's full spec |
| `docs/mapping-algorithm-analysis.md` | Original Algorithm/ review |
| `docs/benchmarks/` | Committed score tables |
| `docs/assignment-compliance-pickup.md` | Submission / packaging (orthogonal) |
| `docs/ex2-grading-handoff.md` | ALG28 hang class still relevant to frontier passability |
