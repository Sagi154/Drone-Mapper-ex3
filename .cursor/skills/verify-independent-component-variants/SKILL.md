---
name: verify-independent-component-variants
description: >-
  Docker-builds independence harness targets and runs VAR-01..04 check scripts
  (foreign host, foreign MissionControl findings dump, adversarial containment,
  optional baseline lawnmower), then reports PASS/FAIL/SKIP per variant. Use when
  verifying plugin independence after Algorithm/MissionControl/Simulator changes,
  before submission, or when asked to re-run independent-component-variants checks.
disable-model-invocation: true
---

# Verify Independent Component Variants

Re-runs the black-box independence harness from
`docs/superpowers/specs/2026-08-28-independent-component-variants-design.md`
(plan: `docs/superpowers/plans/2026-08-28-independent-component-variants.md`).

This is **not** the instructor-catalog orchestrator
(`verify-instructor-test-catalog`). Catalog IDs stay there; this skill keys by
**VAR-01 … VAR-04**.

## Status vocabulary

| Status | Meaning |
|--------|---------|
| **PASS** | Script exited 0; no crash (`exit < 128`); VAR asserts met |
| **FAIL** | Script non-zero, crash, or timeout |
| **SKIP** | User skipped that variant (e.g. `--skip-baseline`) |

VAR-02 is **diagnostic**: script PASS means crash-free + findings dumped. Do **not**
auto-fail on low score / Empty=0 (foreign hits-only by design). Paste findings path
into the report. Hits-only step inflation was former Known Issues #20 (removed after
project B one-scan-per-step). Do not expect `docs/known-issues.md` to still list it.

VAR-03 regression canary: former Known Issues #21 (removed — hang class is structurally
gone). `bad_scan` must finish within timeout. Timeout on `bad_scan` → FAIL.

## Prerequisites

- Repo root = `Drone-Mapper-ex3/`
- Docker image `drone-mapper-ex3-dev`
- Fixtures / host already in tree (`Simulator/tests/hosts/skeleton_host/`,
  `Simulator/tests/fixtures/foreign_*`, `adversarial_*`, `baseline_*`,
  `Simulator/tests/manual/check_*.sh`)

## Optional switches (ask once if unclear)

| Switch | Effect |
|--------|--------|
| (default) | VAR-01 + VAR-02 + VAR-03 (skip VAR-04 — ~5 min alone) |
| `--with-baseline` | Also run VAR-04 `check_baseline_algorithm.sh` |
| `--only-var01` / `--only-var02` / `--only-var03` / `--only-var04` | Single variant |
| `--skip-build` | Assume `build/default` already has needed binaries |
| `--report-path PATH` | Write markdown report (default: chat only; optional file under `docs/`) |

## Procedure

Announce: “Using verify-independent-component-variants.”

### 1. Build (unless `--skip-build`)

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg \
  -v "<repo>:/work" -w /work drone-mapper-ex3-dev bash -lc '
  set -euo pipefail
  cmake --preset default
  cmake --build --preset default --target \
    skeleton_host \
    simulator_207190406_209543255 \
    Algorithm_207190406_209543255 \
    MissionControl_207190406_209543255 \
    foreign_hits_only_mission_control_plugin \
    adversarial_throw_algorithm_plugin \
    adversarial_never_finish_algorithm_plugin \
    adversarial_into_occupied_algorithm_plugin \
    adversarial_bad_scan_orientation_algorithm_plugin \
    adversarial_throw_mission_control_plugin \
    adversarial_empty_mission_control_plugin \
    adversarial_implausible_steps_mission_control_plugin \
    baseline_lawnmower_algorithm_plugin
'
```

On build FAIL → stop; mark all selected variants FAIL (build).

### 2. Run selected checks

Ensure LF line endings / `chmod +x` on the scripts if Windows bind-mount mangled them.

```bash
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg \
  -v "<repo>:/work" -w /work drone-mapper-ex3-dev bash -lc '
  set -euo pipefail
  BD=build/default
  chmod +x Simulator/tests/manual/check_foreign_host.sh \
            Simulator/tests/manual/check_foreign_mission_control.sh \
            Simulator/tests/manual/check_adversarial_plugins.sh \
            Simulator/tests/manual/check_baseline_algorithm.sh
  # VAR-01
  bash Simulator/tests/manual/check_foreign_host.sh "$BD"
  # VAR-02 (findings → /tmp/ex3_verify/foreign_mc_findings.txt inside container)
  bash Simulator/tests/manual/check_foreign_mission_control.sh "$BD"
  # VAR-03
  bash Simulator/tests/manual/check_adversarial_plugins.sh "$BD"
  # VAR-04 only if --with-baseline / --only-var04
  # bash Simulator/tests/manual/check_baseline_algorithm.sh "$BD"
'
```

Record each script’s exit code. For VAR-02, also surface a short summary from the
findings file if reachable (or note path).

### 3. Report template

```markdown
# Independent component variants verification

Date (UTC): …
Branch: …
Switches: …

| Variant | Script | Status | Notes |
|---------|--------|--------|-------|
| VAR-01 | check_foreign_host.sh | PASS/FAIL/SKIP | |
| VAR-02 | check_foreign_mission_control.sh | PASS/FAIL/SKIP | findings (hits-only) |
| VAR-03 | check_adversarial_plugins.sh | PASS/FAIL/SKIP | `bad_scan` canary |
| VAR-04 | check_baseline_algorithm.sh | PASS/FAIL/SKIP | slow ~5m |

Overall: PASS only if every **selected** (non-SKIP) row is PASS.
```

## Evidence map

| Variant | Script | Primary regression risk |
|---------|--------|-------------------------|
| VAR-01 | `check_foreign_host.sh` | Plugins couple to our Map3D/GPS/lidar semantics |
| VAR-02 | `check_foreign_mission_control.sh` | Algorithm assumes Empty-carve / non-null scans |
| VAR-03 | `check_adversarial_plugins.sh` | Containment / `bad_scan` hang |
| VAR-04 | `check_baseline_algorithm.sh` | Competition multi-algorithm wiring |

## Related

- Spec: `docs/superpowers/specs/2026-08-28-independent-component-variants-design.md`
- Plan: `docs/superpowers/plans/2026-08-28-independent-component-variants.md`
- Known Issues: `docs/known-issues.md` (former #20/#21 removed; current #14 is plan-batching)
- Manual README: `Simulator/tests/manual/README.md`
- Catalog orchestrator (different): `verify-instructor-test-catalog`
