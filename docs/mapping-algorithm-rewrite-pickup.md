# Pickup — Mapping algorithm rewrite (A/B/C/D/F)

**Read this first** when continuing the “better algorithm” work. It is the standing handoff:
what landed on branch `algorithm-benchmark-harness`, measured scores, and the ordered
queue. Do not restart from the deadline-driven “patch the current policy” framing in older
docs — quality is the constraint; the A/B/C/D/F roadmap is authoritative (E is the deferred
mp-units conversion).

**Not this file:** assignment packaging / zip / Known Issues excel → still
`docs/assignment-compliance-pickup.md`. That track is separate from the algorithm rewrite.

---

## Branch / git

| Item | Value |
|------|--------|
| Branch | `algorithm-benchmark-harness` (from up-to-date `main` at start of work) |
| Tip (approx) | `docs: record post-F wavefront benchmark against ex2` |

---

## Verdict

**Projects A, B, and C are done.** **D was implemented and measured as a regression** on
runtime (`house_full` / `large_out` killed at 120 s) and on `small_room` (76.37 vs post-C
82.57), while `small_out` jumped to 83.49. **F is done** (WFD clustering + score-aware
scan; honest column recorded 2026-08-31).

Two decisions that still constrain everything below (do not re-litigate without the user):

1. **Optimization target = honest step accounting only** on our MC (carve Empty, pass `latest_scan`).
   Adversarial hits-only MC is a robustness check, not the primary objective.
2. **Success = beat ex2’s recorded 24-cell score bands** (same maps/scorer), not merely
   `mission_score >= 0`.

Full rationale: `docs/superpowers/specs/2026-08-29-mapping-algorithm-roadmap.md`.

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

### Project C — sensor model + clearance

| Artifact | Path |
|----------|------|
| Spec | `docs/superpowers/specs/2026-08-29-sensor-model-clearance-belief-design.md` |
| Plan | `docs/superpowers/plans/2026-08-29-sensor-model-clearance-belief.md` |
| Code | Frontier sphere-box clearance; `UserCommon` BeamMath + LidarCone; gain-gated scan orientations |
| Tests | frontier cm10 cases; `test_lidar_cone.cpp`; updated mapping-algorithm scan expectations |
| Post-C baseline | `docs/benchmarks/2026-08-29-post_c_honest.{csv,md}` |

### Project D — exploration policy (NBV)

| Artifact | Path |
|----------|------|
| Spec | `docs/superpowers/specs/2026-08-29-exploration-policy-nbv-design.md` |
| Plan | `docs/superpowers/plans/2026-08-30-exploration-policy-nbv.md` |
| Outcome | Runtime regression on `house_full` / `large_out` (killed at 120 s); `small_room` 76.37 vs post-C 82.57; `small_out` 83.49 vs 25.16. Candidate-scoring half superseded by F. |

### Project F — wavefront frontier

| Artifact | Path |
|----------|------|
| Spec | `docs/superpowers/specs/2026-08-31-wavefront-frontier-exploration-design.md` |
| Plan | `docs/superpowers/plans/2026-08-31-wavefront-frontier-exploration.md` |
| Code | `WavefrontPlanner` + score-aware executor; NBV candidate API removed |
| Post-F baseline | `docs/benchmarks/2026-08-31-post_f_honest.{csv,md}` (+ adversarial CSV) |

---

## Measured scores (primary numbers)

### Pre-B (batching MC) — `ex2_comparable`

- Score sum **1371.3**, steps **4946**, 0× `MAX_STEPS`, 24 scored.

### Post-C (honest MC + sensor/clearance) — historical optimize-against

- Score sum **1331.0**, steps **9101**, **2× `MAX_STEPS`**, 0 errors.

### Post-D (NBV) — historical

- `-O2` `house_full` / `large_out` killed at 120 s. Profiling `small_room` 76.37; `small_out` 83.49.

### Post-F (WFD) — `honest` ← **current column**

- Score sum **1402.6**, steps **50632**, **2× `MAX_STEPS`**, 0 errors.
- vs ex2: **inside** house_lower, large_room; **below** house_full (mean 6.50), large_out (43.47), small_out (38.36), small_room (70.03).
- Profiling `small_room` **85.60** (≥ post-C 82.57). Profiling `house_full` **25.81** in **65.4 s** (60 s gate **failed**). Profiling `small_out` **34.54** (not the 80s).
- `kMinInformationRate` kept **0.25** (0.10 / 0.25 / 0.50 identical on the six-cell sweep).

### Adversarial (hits-only foreign MC)

- Post-F: score sum **468.2**, steps **1465**, 2× `MAX_STEPS`, 0 errors. Most cells ~28 steps then finish.

**How to re-measure:** see `scripts/benchmark/README.md`. Docker image may need
`apt-get install -y python3-venv python3-pip` once per container. Full 24-cell sweep is
~200 s per column at `num_threads=8` if `house_full` / `large_out` run to ~10k steps.
Do **not** kill cells at 60 s.

---

## Left to do (in order)

### 1. Project E — mp-units on the substrate (next)

Strong-type the remaining frontier / reachability substrate. Packaging / zip / Known
Issues excel remain on the assignment-compliance track.

### 2. After E / packaging

- Open PR(s) for `algorithm-benchmark-harness` — **do not push to `main`**.
- Separate track: zip packaging / Known Issues excel (`assignment-compliance-pickup.md`).

---

## Next session — concrete first steps

1. Read this file + roadmap Project E note.
2. Confirm on `algorithm-benchmark-harness`, `git status` clean.
3. Start **E** (mp-units on the substrate), or packaging if the deadline is the constraint.

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
| `docs/superpowers/specs/2026-08-29-mapping-algorithm-roadmap.md` | Living A/B/C/D/F index |
| `docs/superpowers/specs/2026-08-31-wavefront-frontier-exploration-design.md` | Project F spec |
| `docs/mapping-algorithm-analysis.md` | Original Algorithm/ review |
| `docs/benchmarks/` | Committed score tables |
| `docs/assignment-compliance-pickup.md` | Submission / packaging (orthogonal) |
| `docs/ex2-grading-handoff.md` | ALG28 hang class still relevant to frontier passability |
