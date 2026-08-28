---
name: verify-instructor-test-catalog
description: Orchestrates build, ctest, Simulator/tests/manual/run_all.sh, pre-submission-review, and advcpp-rubric-review into one catalog-ID-keyed report (PASS/FAIL/AMBIGUOUS) against docs/simulator_runtime_test_catalog.md. Use when verifying instructor-catalog coverage end-to-end, before submission, or after closing catalog gaps.
disable-model-invocation: true
---

# Verify Instructor Test Catalog

Runs every automated + skill-based check that maps to
`docs/simulator_runtime_test_catalog.md`, then emits **one consolidated report keyed by
catalog ID** (`CLI-01`, `ZIP-07`, `e14`, …). This is the Phase-B wiring layer that makes the
blind-agent catalog actionable.

**Source of truth for IDs and classifications:** `docs/simulator_runtime_test_catalog.md`
(MANDATORY / OPTIONAL / BONUS / GENUINELY-UNSPECIFIED). Do not invent IDs.

## Status vocabulary (required)

| Status | Meaning |
|--------|---------|
| **PASS** | Evidence shows the mandatory (or claimed optional/bonus) observable was met |
| **FAIL** | Evidence shows a mandatory observable was **not** met |
| **AMBIGUOUS** | Catalog marks the row GENUINELY-UNSPECIFIED / shape-only, **or** no automated/skill evidence exists — **never silently PASS** these |
| **SKIP** | User asked to skip that stage (e.g. `--skip-manual`); list as SKIP, not PASS |

OPTIONAL / BONUS rows: report PASS only if explicitly exercised and green; otherwise AMBIGUOUS
(or SKIP if stage skipped) — do not treat “we didn’t run it” as PASS.

## Prerequisites

- Repo root = `Drone-Mapper-ex3/` (this project).
- Docker image `drone-mapper-ex3-dev` for build / `ctest` / `run_all.sh` (Windows host is not the graded toolchain).
- Skills available: `pre-submission-review`, `advcpp-rubric-review`, `verify-frozen-interfaces`
  (invoked from pre-submission-review §2 / §5 as needed).
- Catalog file present: `docs/simulator_runtime_test_catalog.md`.

## Optional switches (ask once if unclear)

| Switch | Effect |
|--------|--------|
| (default) | Full orchestration including ~1h `run_all.sh` |
| `--skip-manual` | Skip `run_all.sh` (still run build + ctest + skills) |
| `--skip-rubric` | Skip `advcpp-rubric-review` (still include ZIP/e13 grep via pre-submission-review) |
| `--skip-zip` | Skip produced-zip archive step inside pre-submission-review if packaging not ready |
| `--report-path PATH` | Write the consolidated markdown report here (default: `docs/instructor-catalog-verification-report.md`) |

## Procedure

Announce: “Using verify-instructor-test-catalog to produce a catalog-ID report.”

### 1. Build (if needed)

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg \
  -v "<repo>:/work" -w /work drone-mapper-ex3-dev bash -lc '
  cmake --preset default && cmake --build --preset default
'
```

Record: build PASS/FAIL. On FAIL, stop — do not mark runtime rows PASS.

### 2. Unit suite (`ctest`)

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg \
  -v "<repo>:/work" -w /work drone-mapper-ex3-dev bash -lc '
  ctest --test-dir build/default --output-on-failure
'
```

Map failures to catalog IDs using the **Evidence map** below (unit column). Suite green → those IDs PASS unless a later stage contradicts.

### 3. Manual black-box harness

`Simulator/tests/manual/run_all.sh` **already includes** all Point-2 scripts (isolation, multi-plugin,
wall fault, CLI asserts, unwritable dir, competition outdir, etc.). Do **not** invent a parallel list.

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg \
  -v "<repo>:/work" -w /work drone-mapper-ex3-dev bash -lc '
  ./Simulator/tests/manual/run_all.sh /work/build/default
'
```

Expect final line: `run_all.sh: all default-preset checks finished`. Runtime ~1 hour (full
`sim_compose` smoke/collision/threading). On FAIL, note which script failed and mark mapped IDs FAIL.

### 4. Static / structural — invoke `pre-submission-review`

Follow `.cursor/skills/pre-submission-review/SKILL.md` end-to-end (including §5a–5d ZIP-15/13/17/01/04/05).
Also ensure frozen interfaces are checked (§2 or by invoking `verify-frozen-interfaces`).

Map checklist outcomes → ZIP-* / related IDs in the Evidence map.

### 5. Manual rubric — invoke `advcpp-rubric-review`

Follow `.cursor/skills/advcpp-rubric-review/SKILL.md`. That skill returns a **judgment table**, not
boolean PASS/FAIL. In the consolidated report:

- Put `e01`–`e23` findings under status **AMBIGUOUS** (judgment) or note “see findings table”
  unless the user asked for a hard gate.
- Never promote `b*` spreadsheet codes into FAIL for Assignment-3 catalog rows.

### 6. Write the consolidated report

Default path: `docs/instructor-catalog-verification-report.md` (or `--report-path`).

Required sections:

1. **Meta** — date, branch/SHA, switches used, Docker image.
2. **Stage results** — build / ctest / run_all / pre-submission-review / advcpp-rubric-review
   (PASS/FAIL/SKIP + one-line evidence).
3. **Catalog ID table** — one row per ID that this orchestration knows about (Evidence map + any
   UNSPEC-* called out). Columns: `ID | Classification | Status | Evidence`.
4. **Gaps** — catalog mandatory IDs with no evidence source → AMBIGUOUS (not PASS).
5. **Rubric appendix** — embed or link the `advcpp-rubric-review` table if run.

Template row examples:

```markdown
| CLI-05 | MANDATORY | PASS | `check_cli_failures.sh` dual unsupported args |
| UNSPEC-07 | GENUINELY-UNSPECIFIED | AMBIGUOUS | open-questions §9 working assumption only |
| e14 | (rubric) | AMBIGUOUS | advcpp-rubric-review Group D findings |
```

### 7. Stop criteria

- Any **MANDATORY** FAIL → report overall **FAIL** (list blocking IDs).
- All covered MANDATORY rows PASS and no stage FAIL → overall **PASS** (with AMBIGUOUS rows listed separately).
- Do not claim “catalog 100% verified” — only that orchestrated evidence was collected.

## Evidence map (catalog ID → primary evidence)

Use this map when filling the report. If a row has no map entry, default to **AMBIGUOUS**.

### Runtime / CLI / plugins / outputs (manual + unit)

| ID | Primary evidence |
|----|------------------|
| CLI-01 / CLI-02 | `run_smoke_pass.sh` (both modes exit 0) |
| CLI-03 | `check_cli_argument_order.sh` (comparative scramble; competition order still thin → note) |
| CLI-04 / CLI-05 | `check_cli_failures.sh` (`run_and_assert` name substrings) |
| CLI-06 / CLI-07 | `check_cli_failures.sh` nonexistent file / empty folder cases |
| CLI-08 | `check_output_dir_unwritable.sh` |
| PLUGIN-01 / PLUGIN-02 / YAML-OUT-03 | `check_multi_plugin_outputs.sh` |
| PLUGIN-03 | `check_isolation.sh` + distinct fixture `.so`s |
| PLUGIN-04 | `check_all_folder_plugins_fail.sh` |
| OUT-01 | `check_output_dir_collision.sh` |
| OUT-02 | `check_competition_output_dir.sh` |
| OUT-03 | smoke + multi-plugin (dir contents exist) |
| YAML-OUT-01 / YAML-OUT-02 | unit: `test_comparative_report_writer` / `test_competitive_report_writer` (+ smoke reports) |
| YAML-IN-01 | unit: `test_yaml_config_parsers` / smoke load of compose |
| FAULT-02 | `check_wall_collision_fault.sh` + unit `CollisionBlockedThrowContinues` |
| THREAD-01 / THREAD-02 | `check_threading.sh` (+ unit `test_work_distributor`) |
| CLI-OPT-01 / CLI-OPT-02 | smoke without flags / `check_verbose.sh` |

### Static / zip (pre-submission-review + frozen skills)

| ID | Primary evidence |
|----|------------------|
| ZIP-01 / ZIP-04 / ZIP-05 | pre-submission-review §5d produced-zip |
| ZIP-02 / ZIP-03 / ZIP-09 / ZIP-11 / ZIP-12 / ZIP-14 / ZIP-16 | pre-submission-review §§1–4 |
| ZIP-06 | pre-submission-review §5 no binaries |
| ZIP-07 / ZIP-08 | pre-submission-review §2 + `verify-frozen-interfaces` |
| ZIP-13 / ZIP-15 / ZIP-17 | pre-submission-review §5b / §5a / §5c |
| ZIP-10 | build produces SHARED `.so` + executable (build stage) |

### Genuinely unspecified / not auto-verified

Mark **AMBIGUOUS** (never PASS): all `UNSPEC-*`, optional fault rows not exercised (`FAULT-OPT-*`),
`MAP-ALGO` metrics without thresholds, and any MANDATORY row lacking a map entry above.

### Rubric codes

| ID | Primary evidence |
|----|------------------|
| `e01`–`e23` | `advcpp-rubric-review` findings table (judgment → AMBIGUOUS unless user hard-gates) |
| `b*` | Do **not** fail the catalog report on these alone (spreadsheet labels) |

## Anti-patterns

- Marking UNSPEC-* as PASS because “our implementation chose something.”
- Replacing `run_all.sh` with an ad-hoc subset and claiming full harness green.
- Skipping Docker and trusting a Windows-native binary.
- Collapsing the report into “all tests passed” without catalog IDs.
- Editing frozen `common/` / `common_*` during verification.

## Afterward

- If MANDATORY FAILs exist: open/fix issues or Known Issues (`populate-known-issues`) before submission.
- Point 4 does **not** replace packaging — still run `pre-submission-review` zip steps before the real upload.
- Commit the report only if the user asks; default is leave it untracked or overwrite locally.
