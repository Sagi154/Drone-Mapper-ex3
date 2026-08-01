---
name: verify-frozen-interfaces
description: Explicitly verifies that common/, Simulator/common_simulator/, and MissionControl/common_mission_control/ have not been touched — no files added, removed, or moved between them, and no published signature or member changed. Use before committing, before submission, or whenever asked to confirm the frozen-interfaces rule hasn't been violated.
---

# Verify Frozen Interfaces

`.cursor/rules/frozen-interfaces.mdc` freezes `common/`, `Simulator/common_simulator/`, and
`MissionControl/common_mission_control/` — course-published, used **as-is**. This skill runs an
explicit, reproducible check instead of eyeballing a diff.

## 1. Diff against the pristine skeleton

`main` tracks the untouched skeleton copy (see `AGENTS.md`), so compare the working tree against it:

```bash
git diff --name-status main -- common/ Simulator/common_simulator/ MissionControl/common_mission_control/
```

- Empty output → tracked files are clean, go to step 2.
- Any line → **violation**. The status letter says what broke:
  - `A` — a file was added (forbidden, even a "harmless" helper).
  - `D` — a file was removed.
  - `M` — an existing file's contents changed (a signature, a member, anything at all).
  - `R###` — a header was moved, including between two frozen folders.

## 2. Catch new files `git diff` can't see

`git diff <commit>` never reports untracked files, so a newly added-but-unstaged file in `common/` would
pass step 1 silently. Check separately:

```bash
git status --porcelain -- common/ Simulator/common_simulator/ MissionControl/common_mission_control/
```

Any output at all (staged, unstaged, or `??` untracked) is a violation — including a file added deeper
inside these trees than where you were looking.

## 3. Don't rationalize a diff away

If steps 1–2 show anything, it is a violation to **revert**, not to reconcile.
`docs/api-delta-ex2-to-ex3.md` documents what to expect when porting from `Drone-Mapper-ex2` — some of
it is genuine ex2→ex3 skeleton evolution (what changed when the course authored ex3), but its §3/§4
are the opposite: places where `Drone-Mapper-ex2` itself deviated from frozen ex2 headers, which ex3's
skeleton never picked up. Either way, none of it is license for any further change on top of the
skeleton `main` already contains.

## 4. Report the verdict explicitly

- Pass: "✅ Frozen interfaces intact — no diff against `main` and no untracked files in `common/`,
  `Simulator/common_simulator/`, `MissionControl/common_mission_control/`."
- Fail: "❌ Frozen interfaces violated — `<path>`: `<A/M/D/R>`. Revert with `git checkout main -- <path>`
  (or `git rm <path>` / `rm <path>` for an added file)."

## Reference

- `.cursor/rules/frozen-interfaces.mdc` — the rule this checks
- `docs/api-delta-ex2-to-ex3.md` — the porting delta from `Drone-Mapper-ex2`; §1/§2/§5/§6 are genuine
  ex2-skeleton-vs-ex3-skeleton changes, §3/§4 are skeleton-vs-us (our own past deviations to revert)
- `.cursor/skills/pre-submission-review/SKILL.md` step 2 — the same check, folded into the full
  submission pass
