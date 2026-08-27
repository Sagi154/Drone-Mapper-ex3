# Instructor test-catalog follow-up — roadmap

Scoping doc for the 4 follow-up points. Written 2026-08-28. **Point 1 done** (merged to
`main`). **Point 2 plan written** at
`docs/superpowers/plans/2026-08-28-close-instructor-test-catalog-gaps.md` (implementation not
started). Points 3–4 not started.

Inputs already in hand:

- `docs/simulator_runtime_test_catalog.md` — the blind-agent catalog (Phase A output).
- `docs/instructor-test-suite-approach.md` — why/how the catalog was built.
- The gap-diff from this session (catalog rows vs. `Simulator/tests/manual/*.sh` + relevant unit
  tests) — reproduced under Point 2 below.
- `docs/open-questions.md` (8 entries today), `docs/assignment-compliance-pickup.md`,
  `docs/review-error-codes.md`.
- Existing skills: `.cursor/skills/pre-submission-review/SKILL.md`,
  `.cursor/skills/verify-frozen-interfaces/SKILL.md`,
  `.cursor/skills/verify-interfaces-vs-skeleton/SKILL.md`.

Housekeeping that applies to every point below (`.cursor/rules/git-workflow.mdc`): each point is its
own feature branch off updated `main`, Conventional Commits, **human approval before every
`git commit`** — none of this is a green light to commit unattended.

---

## Point 1 — Add UNSPEC-07 to `docs/open-questions.md`

**What it is:** the catalog's `UNSPEC-07` — the docx never states whether a composition's runs are
the full cross product of drones × lidars × missions, or something narrower; the skeleton's
`SimulationCompositionData` shape doesn't settle it either (composition-level `drone_configs` /
`lidar_configs` are siblings of `simulations`, not nested per simulation).

**Work required:**

1. Read the actual current implementation of the composition expansion (likely
   `Simulator/src/RunMatrixOrchestrator*` or wherever `SimulationCompositionData` is turned into a
   run list) to state our **current behavior** accurately — this open-questions entry must record
   what we actually do, not a guess, the same way entries #3/#4 record real working assumptions.
2. Add a new `## 9. How are drones × lidars × missions crossed into composition runs?` section
   following the exact format of the existing 8 entries: quote the ambiguity, state the working
   assumption (= current behavior from step 1), add an **Ask** line for the forum.
3. Cross-check it doesn't contradict `.cursor/rules/simulator-cli-and-outputs.mdc`'s "Both modes run
   **all configurations in the composition YAML** — the full (simulation, mission) pairs × drones ×
   lidars expansion, per plugin" line — if that rule already asserts the full cross product as fact,
   either the rule is the working assumption source (cite it) or the rule itself needs the same
   "working assumption, not a spec fact" caveat added.

**Files touched:** `docs/open-questions.md` only (plus a possible one-line caveat added to
`.cursor/rules/simulator-cli-and-outputs.mdc` if step 3 finds it overstates certainty).

**Size:** small, single-session, no writing-plans overhead needed — just do it directly.

---

## Point 2 — Plan to close the "real gaps" (writing-plans skill)

**Status:** plan file exists —
`docs/superpowers/plans/2026-08-28-close-instructor-test-catalog-gaps.md`. Execute that plan
separately (subagent-driven or inline); do not treat this roadmap section as the task list.

Gaps identified this session (mapped into that plan’s Tasks 1–7 + harness Task 8):

| # | Gap | Catalog rows |
|---|-----|--------------|
| 1 | `check_isolation.sh` uses a renamed copy of our **own** `.so` to approximate "another team's plugin." `valid_algorithm_plugin` / `valid_mission_control_plugin` are already built as test `SHARED` `.so`s, but manual scripts never exercise them through the real CLI. | PLUGIN-03 |
| 2 | `check_collision.sh` name collides with the catalog's wall-collision fault (`FAULT-02`) — it actually tests output-dir collision avoidance. The real mandatory fault (MockMovement throws, caught, no crash) is unit-tested only (`CollisionBlockedThrowContinues` per pickup file), with no black-box script and a misleading filename nearby. | FAULT-02, OUT-01 |
| 3 | `check_cli_failures.sh` only asserts "didn't crash," never that the missing/unsupported **argument names** actually appear in the printed message (the docx's specific graded behavior). Also only ever tries one unsupported arg at a time, never two together to prove "all of them" are named. | CLI-04, CLI-05 |
| 4 | No black-box run exercises 2+ **genuinely distinct** plugin binaries in one comparative/competitive run producing distinct per-plugin outputs — today's fixtures are same-binary copies (`_copy2.so`), and the multi-plugin report shape is only proven via synthetic unit tests. | PLUGIN-01, PLUGIN-02, YAML-OUT-03 |
| 5 | No script asserts the per-plugin "assignment-2-style" output YAML actually exists per successful plugin, named with that plugin's name. | YAML-OUT-03 |
| 6 | No test for: results directory cannot be created (non-writable `mission_control_folder`/`algorithms_folder`) → proper error to screen, no crash. | CLI-08 |
| 7 | No test with scrambled/permuted CLI argument order. | CLI-03 |
| 8 | No script explicitly asserts the exact `competition_<time>` directory name pattern (distinct from `comparative_results_<time>`) the way `check_collision.sh` does for the comparative side. | OUT-02 |

**Work required (when point 2 is actually executed):**

1. Announce use of `superpowers:writing-plans`, follow its header/task structure exactly (Goal,
   Architecture, Tech Stack, Global Constraints, then one `### Task N` per gap above — gaps 1 and 4
   likely merge into one task since gap 4 needs the same "build fixtures as standalone `.so`s" CMake
   work as gap 1).
2. Investigate before writing tasks, not while writing them: how `Simulator/tests/fixtures/*.cpp`
   are currently built (almost certainly compiled straight into a gtest binary, not as a `SHARED`
   library) — the plan needs a concrete new CMake target (mirroring how `Algorithm`/`MissionControl`
   strip the `lib` prefix and export registration symbols per `.cursor/rules/plugin-architecture.mdc`)
   before any shell script can `dlopen` them.
3. Each task must respect existing project rules, not just add scripts ad hoc:
   `.cursor/rules/testing-requirements.mdc` ("keep tests next to the project they exercise," don't
   invent bug-injection ceremony), `.cursor/rules/plugin-architecture.mdc` (`RTLD_LOCAL`, no reload,
   `ENABLE_EXPORTS`), `.cursor/rules/error-handling-logging.mdc` (usage/error text still "our choice,"
   don't hardcode instructor wording), `.cursor/rules/git-workflow.mdc` (branch/commit discipline).
   Each task's exact assertion strings must be **loose** where the assignment doc says "for your
   decision" (e.g. gap 3 should assert the argument *name substring* appears, not a golden string).
4. Every task ends with an independently runnable script/test — no task should require a later task
   to be "done" before it can be verified, per the writing-plans task-sizing rule.
5. Self-review pass at the end: re-check each of the 8 gaps has a task, no placeholders, script names
   consistent with the existing `check_*.sh` convention in `Simulator/tests/manual/`.

**Deliverable:** the plan file itself (not the implementation). Execution is a separate step
(subagent-driven or inline, per the writing-plans handoff prompt) after this plan is reviewed.

---

## Point 3 — Skill coverage for the catalog's "Static/structural checks" and "Not testable by any
automated suite" sections

Current state (checked this session):

- `.cursor/skills/pre-submission-review/SKILL.md` already covers: 5 folders + `UserCommon/` no
  build file (`ZIP-02`, part of `ZIP-03`), `common/` untouched (`ZIP-07`, delegates to
  frozen-interfaces), naming/namespaces/registration macros (`ZIP-09`, `ZIP-11`, `ZIP-12`), build
  file count + compiler flags + no manual installs (`ZIP-03`/`ZIP-16`, `e11`), no binaries in zip
  (`ZIP-06`), README/students.txt/HLD/bonus.txt/Known-Issues presence (part of "Not ready for the
  zip"), and a functional smoke pass (§7) that already overlaps with `Simulator/tests/manual/`.
- `.cursor/skills/verify-frozen-interfaces/SKILL.md` + `verify-interfaces-vs-skeleton` cover `ZIP-07`/
  `ZIP-08` thoroughly (including a from-scratch subagent audit against a freshly-pulled skeleton).
- **Not covered by any skill today:**
  - `ZIP-15` (no `new`/`delete` in source — a grep pass, `e13`/AdvCpp rule, currently only stated as
    a rule in `.cursor/rules/adv-cpp-standards.mdc`, never checked by a skill).
  - `ZIP-13` (mocks — `MockLidar`, `MockGPS`, `MockMovement`, `Map3DImpl`, `MapsComparison` — live
    under `Simulator/src/`, not leaked into `Algorithm/` or `MissionControl/`).
  - `ZIP-17` (staff `inputs/` tree presence/expectations — currently just a note in the catalog).
  - `ZIP-01` (the actual zip's archive name, since `pre-submission-review` checks the working tree,
    not a produced zip file) and `ZIP-04`/`ZIP-05` exact placement checks (root-only, not nested).
  - Everything in the catalog's **manual review items** section: AdvCpp code-quality rubric
    (`e01`–`e23` from `docs/review-error-codes.md`), the qualitative AdvCpp Review Guideline items
    (coherent structure, const-correctness, rule of 3/5, magic numbers, short functions, HLD/readme
    matching code), and the `b*` build/run codes that aren't already exercised by
    `pre-submission-review` §7's functional pass.

**Work required:**

1. **Extend `pre-submission-review`** with the missing deterministic/grep-able static checks:
   a `new`/`delete` grep step (`ZIP-15`), a mocks-placement check (`ZIP-13` — grep for those class
   names outside `Simulator/src/`), an `inputs/` tree note (`ZIP-17`), and a final "produced zip"
   step that actually runs `zip`/inspects the archive for `ZIP-01`/`ZIP-04`/`ZIP-05` instead of only
   checking the working tree (today's skill implicitly assumes working tree == zip contents, which
   is true only if nothing extra gets swept in).
2. **Create one new skill** for the manual/subjective review items — something like
   `.cursor/skills/advcpp-rubric-review/SKILL.md`. This is categorically different from the other
   skills (subjective code judgment, not a deterministic checklist), so it should:
   - Walk the codebase against `docs/review-error-codes.md`'s `e01`–`e23` table one code at a time,
     dispatching a subagent per code group (headers/API codes vs. structure codes vs. HLD codes)
     the same way `gather-instructor-context` dispatches one extractor per document — a single pass
     over the whole tree per code is more reliable than one pass checking all 19 codes at once.
   - Cross-check `HLD.pdf` / `docs/HLD.md` diagrams against the actual class/sequence structure for
     `e14`/`e15`, and the AdvCpp guideline's "HLD and readme must match the code" line generally.
   - Report findings as a table (`code`, file:line, severity per `docs/review-error-codes.md`,
     suggested fix) rather than a pass/fail boolean — these are judgment calls, not a script's exit
     code, and the skill should say so explicitly rather than pretending certainty it doesn't have.
   - Explicitly flag that `b05` (1-minute timeout) and other `b*` codes are review-spreadsheet
     labels, not Assignment-3 prose — same caution the catalog already applies — so this skill
     doesn't accidentally promote them into hard requirements.
3. Decide whether `docs/known-issues-guidelines.md` / `populate-known-issues` already fully covers
   the "Known Issues process" manual item, or needs a cross-reference added from the new skill.

**Files touched:** `.cursor/skills/pre-submission-review/SKILL.md` (extended),
new `.cursor/skills/advcpp-rubric-review/SKILL.md`, possibly a one-line cross-reference added to
`.cursor/skills/populate-known-issues/SKILL.md` and to `AGENTS.md`'s skills table.

---

## Point 4 — Orchestrator skill that runs everything verifying the catalog

Only makes sense **after** points 1–3 land (needs the closed gaps from Point 2 and the new/extended
skills from Point 3 to have something to call).

**What it should do**, e.g. `.cursor/skills/verify-instructor-test-catalog/SKILL.md`:

1. Build the project (the existing build presets) if not already built.
2. Run the unit suite (`ctest`) — covers `YAML-OUT-01`/`YAML-OUT-02` and others proven at unit level.
3. Run `Simulator/tests/manual/run_all.sh` **plus** the new scripts added by Point 2's plan (update
   `run_all.sh` itself to include them, or have the orchestrator call the expanded list directly).
4. Invoke `pre-submission-review` (as extended by Point 3) for the static/structural section.
5. Invoke the new `advcpp-rubric-review` skill for the manual-review section.
6. Produce **one consolidated report keyed by catalog ID** (`CLI-01`, `ZIP-07`, `e14`, …) pulling
   directly from `docs/simulator_runtime_test_catalog.md`'s IDs, so the output is traceable back to
   the exact catalog row rather than a generic "tests passed" summary — this is the actual point of
   having built the catalog in the first place.
7. Clearly separate PASS / FAIL / AMBIGUOUS-per-catalog (genuinely-unspecified rows stay AMBIGUOUS,
   never silently PASS) in the final report, matching the catalog's own MANDATORY/OPTIONAL/BONUS/
   GENUINELY-UNSPECIFIED classification.

**Files touched:** new `.cursor/skills/verify-instructor-test-catalog/SKILL.md`, a small update to
`Simulator/tests/manual/run_all.sh` (or equivalent) to include Point 2's new scripts, and a row added
to `AGENTS.md`'s skills table.

---

## Suggested execution order

1. Point 1 (small, standalone, unblocks nothing else but is quick).
2. Point 2's **plan** (writing-plans doc) — review it before any implementation starts.
3. Point 3 (skill work) — independent of Point 2's implementation, can happen in parallel once
   scoped.
4. Execute Point 2's plan (subagent-driven or inline, per the plan's own handoff).
5. Point 4 — only once 2's implementation and 3 both exist to be orchestrated.
