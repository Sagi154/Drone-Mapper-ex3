---
name: verify-interfaces-vs-skeleton
description: Pulls the latest ex_3_skeleton from its upstream remote, then dispatches an explore subagent to exhaustively diff every header under common/, Simulator/common_simulator/, and MissionControl/common_mission_control/ against that freshly-updated skeleton checkout — checking method signatures, class contracts, enums, and CMake targets, not just a text diff. Use for a deep from-scratch audit of frozen-interface compliance (e.g. before submission, after a rebase/merge, or whenever asked to confirm ex3 hasn't changed any skeleton interface), especially as a cross-check independent of git history on `main` and of whether the local skeleton clone might be outdated.
---

# Verify Interfaces vs. Skeleton

`.cursor/rules/frozen-interfaces.mdc` freezes `common/`, `Simulator/common_simulator/`, and
`MissionControl/common_mission_control/`. `.cursor/skills/verify-frozen-interfaces/SKILL.md` checks this
fast via `git diff main`, which is correct *if* `main` reliably still equals the pristine skeleton. This
skill instead re-derives the answer from the actual original skeleton directory, independent of git
history — use it when you want a from-scratch audit, not a trust-`main` shortcut.

## 1. Locate the pristine skeleton checkout

It lives as a sibling checkout, not inside this repo. Find it once per machine:

```bash
find / -maxdepth 6 -iname "*ex_3_skeleton*" 2>/dev/null
```

Expect something like `<workspace-root>/ex_3_skeleton`. If it's missing, stop and tell the user — there
is nothing to diff against.

## 2. Pull the latest skeleton before diffing

The skeleton is itself a git checkout tracking the course's upstream repo (`origin/main`). The course can
push amendments after the initial publish, so a stale local clone can silently hide a real interface
change (or flag a false one). Always refresh it first — never diff against whatever happens to be on
disk:

```bash
cd <path to ex_3_skeleton> && git fetch origin && git status -sb && git log --oneline HEAD..origin/main
```

- If `git status -sb` shows local modifications/untracked files in the skeleton checkout itself, stop —
  that checkout is no longer pristine and can't be trusted as the baseline; get a clean clone instead.
- If `git log --oneline HEAD..origin/main` prints anything, the upstream skeleton has moved forward:
  fast-forward it (`git merge --ff-only origin/main`, or `git pull --ff-only`) before continuing, and
  call out in your final report that the skeleton itself changed upstream and which commits landed —
  this also means `.cursor/skills/verify-frozen-interfaces/SKILL.md`'s assumption that `main` in
  Drone-Mapper-ex3 already equals the skeleton may now be **stale** until `main` is re-synced to match.

Only proceed to the audit once the skeleton checkout is confirmed clean and up to date with its remote.

## 3. Dispatch a subagent to do the full audit

Do not do this comparison yourself inline — dispatch it with the `Task` tool
(`subagent_type: "explore"`, thoroughness "very thorough") so the enumeration and per-file reasoning
happen in an isolated context. Use this prompt template, filling in the two real paths:

```
You must verify that the current implementation in `<path to Drone-Mapper-ex3>` has NOT changed ANY
public interface compared to the original skeleton in `<path to ex_3_skeleton>`.

1. Get an overview of both directory trees (excluding `build/`, `.git/`, vcpkg dirs) to map
   corresponding files between the two projects.
2. Identify all interface files in the skeleton: headers under `common/`, `Simulator/common_simulator/`,
   and `MissionControl/common_mission_control/` — class/struct declarations, method signatures, enums,
   templates, abstract base classes, namespaces, and the published CMake target(s) (e.g. `common::common`).
3. For every skeleton header, find its counterpart in Drone-Mapper-ex3 (it may have moved — search by
   filename and by class name) and diff them, focusing on: renamed/removed classes or methods, changed
   signatures (return type, params, order, const-ness, defaults, virtual/override qualifiers), changed
   access specifiers, changed public member types/names, namespace changes, template parameter changes,
   reordered/renamed/removed enum values, and new pure-virtual methods added to an existing interface.
   New files/classes/methods that don't exist in the skeleton at all are fine — only flag
   removals/modifications of things the skeleton already defined.
4. Use `diff -u` and `cmp -s` between corresponding files, but also actually read each header's contract
   — reformatting/moved includes with no semantic change is fine; a subtle signature change hidden in an
   otherwise-cosmetic diff is not.
5. Also check whether the published CMake target(s) (name, alias, what they link/expose) changed.

Produce a final report: a table of every skeleton header/interface file → its ex3 counterpart (or
"REMOVED") → UNCHANGED or CHANGED (with exact before/after), and a final verdict on whether ANY
interface changed. Be exhaustive; report every change no matter how minor (default param added, param
renamed, new virtual method, etc.).
```

## 4. Report the verdict explicitly

Relay the subagent's final verdict directly:

- Pass: state that every skeleton header/interface file and CMake target were found unchanged, and list
  the frozen directories checked.
- Fail: list every changed file with the exact before/after, and say revert with
  `git checkout main -- <path>` (or copy the file back from `ex_3_skeleton`) — a diff here is a violation
  to undo, not to reconcile.

## Reference

- `.cursor/rules/frozen-interfaces.mdc` — the rule this checks
- `.cursor/skills/verify-frozen-interfaces/SKILL.md` — the fast git-based check against `main`
- `docs/api-delta-ex2-to-ex3.md` — legitimate ex2→ex3 skeleton evolution, for context when explaining a
  finding (not license for any change on top of the ex3 skeleton itself)
