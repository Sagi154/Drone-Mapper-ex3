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
| Tip (approx) | outdoor Empty-carve (small_out volume + short-lidar horizon; house_full unchanged) |

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
| Docs | HLD, mapping-algorithm-analysis portability section, VAR-02 table (former known-issues #20/#21 removed 2026-09-01) |
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

### Post-F (WFD) — historical

- Score sum **1402.6**, steps **50632**, **2× `MAX_STEPS`**, 0 errors.
- vs ex2: **inside** house_lower, large_room; **below** house_full (mean 6.50), large_out (43.47), small_out (38.36), small_room (70.03).
- Three `house_full` variants at **0.06** (two in 3 steps, one at 10000).

### Score-aware nav (post-F follow-up) — historical

- Score sum **1589.4**, steps **92452**, **0× `MAX_STEPS`**, 0 errors.
- CSV/md: `docs/benchmarks/2026-08-31-score_aware_nav.{csv,md}`.
- vs ex2: **inside** house_lower, large_room (94.47); **below** house_full (30.46, was 6.50), large_out (59.84), small_out (30.46), small_room (82.12).
- `house_full` 0.06×3 is gone. Profiling small+short **48.33**. Large-drone `small_room` **85–86** (was 49–59).
- `kMinInformationRate` / empty-stay cut **did not bind** on the 10k-step cells.

### Outdoor Empty-carve — `honest` ← **current column**

- Score sum **1793.4**, steps **92464**, **3× `MAX_STEPS`**, 0 errors.
- CSV/md: `docs/benchmarks/2026-08-31-outdoor-empty-carve.{csv,md}`.
- vs ex2: **inside** house_lower, large_room; **below** house_full (30.46, gap 25.5), large_out (68.50, gap 11.5), small_out (72.81, gap 2.2), small_room (82.11).
- `small_out` small-drone cells **81–82** (inside 75–89); large+short **90.9**. Group drag is large+long **36**.
- `large_out` large+short **85.6** (inside 80–88). Long-lidar horizon volume on the 300 cm cube was measured and **reverted** (73→39).
- `house_full` identical to score-aware nav (15–48). Rooms / `house_lower` 100 did not regress.
- Do **not** globally unmask cone gain (rooms collapse). `small_out` (200 cm cube) may volume-carve non-downward cones; `large_out` long lidar stays masked.

### Adversarial (hits-only foreign MC)

- Post-F: score sum **468.2**, steps **1465**, 2× `MAX_STEPS`, 0 errors. Most cells ~28 steps then finish.

**How to re-measure:** see `scripts/benchmark/README.md`. Docker image may need
`apt-get install -y python3-venv python3-pip` once per container. Full 24-cell sweep is
~200 s per column at `num_threads=8` if `house_full` / `large_out` run to ~10k steps.
Do **not** kill cells at 60 s.

---

## Left to do (in order)

### 1. Close the remaining ex2-band gaps (this is the standing work)

Do **not** start Project E until house_full / outdoor / small_room means are inside
or clearly approaching the ex2 bands. E is type cleanup; it will not raise scores.

Diagnosed post-F failure modes (2026-08-31):

| Symptom | Cause | Fix landed / still open |
|---------|--------|-------------------------|
| `house_full` large drone 0.06 in **3** steps | Spawn world-z = `max_height` (300 cm). r=7.5 sphere clips +Z OOB; `start_passable` false; 3 low-rate replans; footprint-only score. `house_lower` 100 is the empty-universe artifact (spawn above that box). | OOB no longer blocks an in-bounds centre. Occupied still does. |
| Large drone quits after painting a floor | r=7.5 hits Occupied face neighbour (nearest 5). Planner returned invalid instead of `findUnstickPath`. | Unstick plan when start is blocked. |
| `house_full` 10k steps, score stuck ~15–37 | Downward scans at the ceiling paint PO (`z_min`) into the slab (pass-2). Gating those scans then left start as the cheapest frontier (Unmapped still below) so we never descended. | Gate downward cones at `max_height`; force a one-step descend when Unmapped is below. |
| 10k steps that do not raise score | Cluster `cell_count` was the Unmapped **volume**, so `kMinInformationRate` never bound. | Rank / terminate on Empty surface size. Rate floor still rarely binds on outdoor (surface stays large). |
| Outdoor far below D's 83 | Frontier-mask + no volume carving. Unmasked / "open-look" gain repeats D's pass-2 hit on rooms — do not re-enable globally. | **Landed (gated):** `small_out` volume-carves non-downward cones + 4-scan cap (mean 30→73). `large_out` short lidar horizon volume (large+short 67→86). Long lidar on the 300 cm cube stays masked. |
| `small_room` large 49–59 | Same Occupied-floor / OOB trap as house. | Unstick + OOB → ~85–86 (near band). |

Current honest sum **1793.4** (score-aware nav 1589.4). Still below ex2 on house_full (56–62),
large_out (80–88), small_out mean (75–89, gap 2.2), small_room mean (87–90).

### 2. Project E — mp-units on the substrate (after scores)

Strong-type the remaining frontier / reachability substrate. Packaging / zip / Known
Issues excel remain on the assignment-compliance track.

### 3. After E / packaging

- Open PR(s) for `algorithm-benchmark-harness` — **do not push to `main`**.
- Separate track: zip packaging / Known Issues excel (`assignment-compliance-pickup.md`).

---

## Next session — concrete first steps

1. Read this file. Confirm `algorithm-benchmark-harness`.
2. Re-measure Release honest if HEAD moved (`build/opt`, `scripts/benchmark/run_benchmark.py --columns honest`).
3. Do not globally unmask cone gain. `small_out` already volume-carves non-downward cones.
   Next levers: get `house_full` off the ceiling layer into the rest of the volume (prefer-descend
   did not bind); recover `large_out` small+long toward F's 70; lift `small_out` large+long (36).
4. Project E / zip only after the score bar, or if the user says the deadline is the constraint.

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
