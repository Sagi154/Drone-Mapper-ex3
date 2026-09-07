# Verify submission readiness — design

**Date:** 2026-09-06  
**Status:** Implemented 2026-09-06 (skill + AGENTS.md pointer).  
**Goal:** One named skill that sequences the existing verification skills into a stage dashboard, without merging ID namespaces or reimplementing child procedures.

## Problem

We have several independent verify skills (`verify-instructor-test-catalog`, `verify-independent-component-variants`, `verify-cell-runtime`, `verify-frozen-interfaces`, `verify-interfaces-vs-skeleton`). Before claiming “we’re good,” an agent (or human) must remember which to run and in what order. Folding them into the catalog orchestrator would mix catalog IDs with VAR rows, per-cell walls, and skeleton diffs, and would change what “catalog FAIL” means.

## Non-goals

- Do not add stages to `verify-instructor-test-catalog`. Catalog IDs stay there.
- Do not reimplement child skill procedures, Docker commands, or evidence maps.
- Do not invent catalog IDs, VAR IDs, or cell names in this skill.
- Do not include zip packaging / Known Issues excel as a verification stage (Afterward pointer only).
- Do not add `pre-submission-review` or `advcpp-rubric-review` as extra stages — they already run inside the catalog skill.
- No scripts; this is orchestration text only.

## Deliverable

Project skill: `.cursor/skills/verify-submission-readiness/SKILL.md`

- `disable-model-invocation: true` (same as catalog / VAR skills).
- Description: WHAT + WHEN, third person, trigger terms (“submission ready”, “all checks”, “final gate”).
- Also: one row in `AGENTS.md` skills table; one-line “not the meta-skill” note on `verify-instructor-test-catalog`.

## Architecture

Thin meta-orchestrator. Invoke child skills by name; copy their overall stage status into a dashboard.

Default order (cheap gates first):

1. **Frozen interfaces** — always run `verify-frozen-interfaces` first (unless the user explicitly skipped it in free text after the switch prompt). If `--deep-skeleton`, also run `verify-interfaces-vs-skeleton`. If the fast check fails without that flag, run the deep check once to see whether `main` is stale vs upstream skeleton, then still **Stop** on frozen FAIL.
2. **Instructor catalog** — `verify-instructor-test-catalog` (build, `ctest`, `run_all.sh`, `pre-submission-review`, rubric). Catalog IDs stay in that child report.
3. **Independence** — `verify-independent-component-variants` (`VAR-01`…`VAR-04`).
4. **Cell runtime** — `verify-cell-runtime` last (Release, serial 24-cell). WARN ≠ FAIL.

## Hard gate: ask switches before running

After announcing “Using verify-submission-readiness,” and **before** any Docker/build/child skill run, the agent **must** show the switch table and wait for an explicit user choice.

Do not assume the default. A full run is ~1.5–2 hours; silent default is the failure mode this gate exists to prevent.

**Skip the prompt only if** the invoking user message already named the switches (e.g. “run verify-submission-readiness `--skip-catalog`”). Then echo the parsed switches in one line and proceed.

If the user answers “default”, “all four”, “all”, or “go”, that **is** explicit consent for the default (all four stages, no catalog skips, no `--deep-skeleton`, default report path).

Show this table (keep it in the skill so the agent pastes it, not paraphrases):

| Switch | Effect |
|--------|--------|
| (default) | All four stages, including ~1h catalog `run_all.sh` |
| `--skip-catalog` | Skip `verify-instructor-test-catalog` |
| `--skip-variants` | Skip `verify-independent-component-variants` |
| `--skip-runtime` | Skip `verify-cell-runtime` |
| `--deep-skeleton` | Run `verify-interfaces-vs-skeleton` instead of (or after a failed) fast frozen check |
| `--report-path PATH` | Write the dashboard markdown here (default: chat + `docs/submission-readiness-report.md`) |
| `--skip-manual` | Forwarded to catalog only (skip `run_all.sh`) |
| `--skip-rubric` | Forwarded to catalog only |
| `--skip-zip` | Forwarded to catalog only |

There is no `--skip-frozen`. Frozen is the cheap gate; omit it only if the user explicitly says to skip it in free text after seeing the table.

Ask once. Do not re-prompt after the run starts. Catalog-only flags are forwarded, not reimplemented.

## Stop / continue

| Condition | Behavior |
|-----------|----------|
| Frozen FAIL | **Stop.** Revert; do not run later stages. |
| Build unusable (catalog build FAIL, or no binaries) | Skip remaining stages that need binaries; mark them FAIL (build), not SKIP. |
| Catalog FAIL (tests/zip/rubric, binaries still usable) | Continue variants + runtime so the dashboard still has that evidence. |
| Catalog AMBIGUOUS rows | Do not fail overall. |
| Cell WARN | Do not fail overall. |
| VAR-02 diagnostic (hits-only scores) | Script PASS = crash-free; do not fail overall on low score / Empty=0. |

Overall **FAIL** if any **selected** hard-gate stage FAILs. SKIP rows do not fail overall. A default run that skipped a stage without a user switch is not PASS.

## Report

Dashboard only — one row per stage, not a merged ID table.

```markdown
# Submission readiness

Date (UTC): …
SHA: …
Switches: …

| Stage | Skill | Status | Notes |
|-------|-------|--------|-------|
| Frozen | verify-frozen-interfaces | PASS/FAIL/SKIP | or deep-skeleton |
| Catalog | verify-instructor-test-catalog | PASS/FAIL/SKIP/AMBIGUOUS | pointer to catalog report |
| Variants | verify-independent-component-variants | PASS/FAIL/SKIP | VAR-01…04 |
| Runtime | verify-cell-runtime | PASS/FAIL/SKIP | WARN listed, not FAIL |

Overall: PASS only if every selected (non-SKIP) hard-gate stage is PASS.
```

Do not collapse to “all tests passed.” Point at child reports; do not copy their ID tables.

## Afterward (not a stage)

Packaging still requires `pre-submission-review` zip steps (already inside catalog unless `--skip-zip` / `--skip-catalog`) and Known Issues excel via `populate-known-issues`. This skill does not produce the zip.

## Anti-patterns

- Silent default (no switch table, no wait).
- Folding child evidence into catalog IDs.
- Re-running `pre-submission-review` as its own stage.
- Treating cell WARN or catalog AMBIGUOUS as overall FAIL.
- Skipping VAR-04 on a default variants stage.
- Editing frozen `common/` / `common_*` during verification.

## Implementation notes

- Follow sibling `verify-*` skill shape: YAML frontmatter, announce line, switches table, procedure, report template, anti-patterns.
- Invoke children by skill name; do not paste their Docker blocks into this file.
- Keep `SKILL.md` well under 500 lines (target ~150).
