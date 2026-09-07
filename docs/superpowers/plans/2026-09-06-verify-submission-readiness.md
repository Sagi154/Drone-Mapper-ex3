# Verify Submission Readiness Skill Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a thin `verify-submission-readiness` project skill that sequences existing verify skills into a stage dashboard, with an ask-first switch prompt.

**Architecture:** One new `SKILL.md` that invokes child skills by name (no Docker blocks, no merged ID tables). Pointers only: one `AGENTS.md` table row and a one-line “not the meta-skill” note on `verify-instructor-test-catalog`.

**Tech Stack:** Cursor project skill (YAML frontmatter + markdown). No scripts, no C++.

## Global Constraints

- Repo: `Drone-Mapper-ex3/`. Skill path: `.cursor/skills/verify-submission-readiness/SKILL.md`.
- `disable-model-invocation: true`.
- Do not add stages to `verify-instructor-test-catalog`. Do not paste child Docker/evidence maps.
- Do not invent catalog IDs / VAR IDs / cell names. No `pre-submission-review` or `advcpp-rubric-review` as extra stages.
- Spec: `docs/superpowers/specs/2026-09-06-verify-submission-readiness-design.md`.
- Git: feature branch from updated `main`; human approval before any commit.
- `SKILL.md` target ~150 lines, hard cap 500.

---

### Task 1: Feature branch from updated `main`

**Files:** none (git only)

- [ ] **Step 1: Sync `main` and create branch**

```bash
git checkout main
git pull
git checkout -b add-verify-submission-readiness-skill
```

Expected: branch `add-verify-submission-readiness-skill`; untracked spec file still present.

- [ ] **Step 2: Do not commit yet** — later tasks write files; commit only if the user asks.

---

### Task 2: Write `verify-submission-readiness` skill

**Files:**
- Create: `.cursor/skills/verify-submission-readiness/SKILL.md`

**Interfaces:**
- Consumes: child skills by name only — `verify-frozen-interfaces`, `verify-interfaces-vs-skeleton`, `verify-instructor-test-catalog`, `verify-independent-component-variants`, `verify-cell-runtime`.
- Produces: stage dashboard (Frozen / Catalog / Variants / Runtime); switch prompt table copied from the spec.

- [ ] **Step 1: Create directory and `SKILL.md`** with this exact body (do not add Docker blocks):

```markdown
---
name: verify-submission-readiness
description: >-
  Sequences frozen-interface, instructor-catalog, independent-component-variant,
  and cell-runtime checks into one stage dashboard. Use when claiming
  submission ready, running a final gate across all verify skills, or asking
  whether we are good across catalog / VAR / runtime / frozen checks.
disable-model-invocation: true
---

# Verify Submission Readiness

Thin meta-orchestrator. Invoke child skills by name; copy each child's overall
status into a **stage dashboard**. Do not merge catalog IDs, VAR IDs, or
per-cell walls into one table. Do not reimplement child procedures.

This is **not** `verify-instructor-test-catalog`. Catalog IDs stay there; this
skill sequences that skill as one stage.

Announce: “Using verify-submission-readiness.”

## Hard gate — ask switches before running

After the announce line, and **before** any Docker/build/child skill run, show
the table below **verbatim** and wait for an explicit user choice.

Do not assume the default. A full run is ~1.5–2 hours.

**Skip the prompt only if** the invoking user message already named the
switches (e.g. `verify-submission-readiness --skip-catalog`). Then echo the
parsed switches in one line and proceed.

If the user answers “default”, “all four”, “all”, or “go”, that **is** explicit
consent for the default (all four stages, no catalog skips, no `--deep-skeleton`,
default report path).

Ask once. Do not re-prompt after the run starts.

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

There is no `--skip-frozen`. Frozen is the cheap gate; omit it only if the user
explicitly says to skip it in free text after seeing the table.

Catalog-only flags are forwarded into `verify-instructor-test-catalog`, not
reimplemented here.

## Stages (cheap gates first)

1. **Frozen** — always run `verify-frozen-interfaces` first (unless the user
   explicitly skipped it in free text after the prompt). If `--deep-skeleton`,
   also run `verify-interfaces-vs-skeleton`. If the fast check fails without
   that flag, run the deep check once to see whether `main` is stale vs upstream
   skeleton, then still **Stop** on frozen FAIL.
2. **Catalog** — `verify-instructor-test-catalog` (build, `ctest`, `run_all.sh`,
   `pre-submission-review`, rubric). Catalog IDs stay in that child report.
   Do **not** run `pre-submission-review` or `advcpp-rubric-review` as extra
   stages.
3. **Variants** — `verify-independent-component-variants` (`VAR-01`…`VAR-04`).
   Do not skip VAR-04 on a default variants stage.
4. **Runtime** — `verify-cell-runtime` last (Release, serial 24-cell). WARN ≠ FAIL.

## Stop / continue

| Condition | Behavior |
|-----------|----------|
| Frozen FAIL | **Stop.** Revert; do not run later stages. |
| Build unusable (catalog build FAIL, or no binaries) | Skip remaining stages that need binaries; mark them FAIL (build), not SKIP. |
| Catalog FAIL (tests/zip/rubric, binaries still usable) | Continue variants + runtime so the dashboard still has that evidence. |
| Catalog AMBIGUOUS rows | Do not fail overall. |
| Cell WARN | Do not fail overall. |
| VAR-02 diagnostic (hits-only scores) | Script PASS = crash-free; do not fail overall on low score / Empty=0. |

Overall **FAIL** if any **selected** hard-gate stage FAILs. SKIP rows do not
fail overall. A default run that skipped a stage without a user switch is not
PASS.

## Report

Dashboard only — one row per stage. Write to chat and, unless the user chose
otherwise, `docs/submission-readiness-report.md` (or `--report-path`). Commit
the report only if the user asks.

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

Do not collapse to “all tests passed.” Point at child reports; do not copy
their ID tables.

## Afterward (not a stage)

Packaging still requires `pre-submission-review` zip steps (already inside
catalog unless `--skip-zip` / `--skip-catalog`) and Known Issues excel via
`populate-known-issues`. This skill does not produce the zip.

## Anti-patterns

- Silent default (no switch table, no wait).
- Folding child evidence into catalog IDs.
- Re-running `pre-submission-review` as its own stage.
- Treating cell WARN or catalog AMBIGUOUS as overall FAIL.
- Skipping VAR-04 on a default variants stage.
- Editing frozen `common/` / `common_*` during verification.
- Pasting child Docker commands or evidence maps into this file.
```

- [ ] **Step 2: Verify the file**

```bash
# from repo root
test -f .cursor/skills/verify-submission-readiness/SKILL.md
rg -n "docker run" .cursor/skills/verify-submission-readiness/SKILL.md
rg -n "Using verify-submission-readiness" .cursor/skills/verify-submission-readiness/SKILL.md
rg -n "disable-model-invocation: true" .cursor/skills/verify-submission-readiness/SKILL.md
```

Expected: file exists; `docker run` has no matches; announce line and `disable-model-invocation` present.

---

### Task 3: Pointers in `AGENTS.md` and catalog skill

**Files:**
- Modify: `AGENTS.md` (skills table, after `verify-instructor-test-catalog` row)
- Modify: `.cursor/skills/verify-instructor-test-catalog/SKILL.md` (after the opening paragraph, before “Source of truth”)
- Modify: `docs/superpowers/specs/2026-09-06-verify-submission-readiness-design.md` status line

- [ ] **Step 1: Add this row to the `AGENTS.md` skills table** immediately after the `verify-instructor-test-catalog` row:

```markdown
| `verify-submission-readiness` | Final-gate meta-orchestrator: ask switches, then frozen + catalog + VAR-01…04 + cell-runtime → one stage dashboard. Use when claiming submission ready or asking whether all verify checks are green. Not a catalog-ID report. |
```

- [ ] **Step 2: Insert this paragraph** in `.cursor/skills/verify-instructor-test-catalog/SKILL.md` after line 12 (after “actionable.”), before “**Source of truth”:

```markdown
This is **not** the submission-readiness meta-skill
(`verify-submission-readiness`). Catalog IDs stay here; that skill sequences
this as one stage.
```

- [ ] **Step 3: Update spec status** first line block from `Not implemented yet` to `Implemented 2026-09-06 (skill + AGENTS.md pointer).`

- [ ] **Step 4: Verify pointers**

```bash
rg -n "verify-submission-readiness" AGENTS.md .cursor/skills/verify-instructor-test-catalog/SKILL.md
```

Expected: at least one hit in each file.

- [ ] **Step 5: Commit only if the user asks.** Proposed message if they do:

```text
docs: add verify-submission-readiness meta-skill
```
