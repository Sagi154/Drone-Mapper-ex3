# Known Issues file

From `context/Known Issues.docx`. **Optional but grade-improving** — submit as an exported Excel file
alongside the assignment. The staff provide a Google Sheet template to clone.

Working rows (markdown, not the excel): `docs/known-issues.md`.
Agent skill for adding/updating rows: `.cursor/skills/populate-known-issues/SKILL.md`.

## Why it is worth filling in

- A documented known issue costs **no more** points than the same undocumented issue, and for small
  things documenting it may cost **nothing at all**.
- Points are not deducted for something the staff did not plan to deduct for, just because it is listed.
- Conversely: if something *should* have been listed and is not, they may deduct **more**, because it
  suggests that part was never tested.
- It doubles as our own TODO list for the next iteration.

Fill it in **English**. Questions go to the Moodle Ex 3 forum.

**Do not treat Ex1 leftovers in `context/Submission Guidelines - Advanced Topics Assignments - 2026B.docx` as Ex3 gaps.** That file still describes `drone_mapper`, `map_output.txt`, three `original_output` folders, and zip `ID1_ID2.zip`. Assignment 3’s own docx wins for zip name (`ex3_<id1>_<id2>.zip`), CLI, `students.txt`, and the 5-folder layout. Compiler flags, HLD PDF, readme names/IDs, no binaries, and forum-approved libs still apply.

## Required columns

`#` · `Type` · `Sub Type` · `Description` · `Severity` · `Reproducibility` · `Reproduce steps` ·
`Reason` · `Reason notes`

### Type / Sub Type

| Type | Sub types |
|------|-----------|
| **Feature** — a requirement is entirely or partially missing | Feature Missing · Feature Partially Missing · Feature developed in a different way |
| **Bug** — bad runtime behavior | Program crash · Program misbehave |
| **Coding** — imperfect code | Missing a required class · Redundant class · Code duplication · Function too long · Class doing another class's job · Spread logic · Code too complex · Hard-coded numbers instead of enums/constants · Bad names · Other |

For hard-coded numbers and bad names there is no need to document per case — one row covers it.

### Severity

| Level | Meaning |
|-------|---------|
| High | Submission is very poor — won't function in many cases, misses a major requirement, or the code is awful (e.g. everything in one class) |
| Medium | Won't function in some cases, misses a minor requirement, or the code is not good (e.g. a very long, complex function) |
| Low | Affects very rare cases, or a small code issue, or a very minor missing requirement |

### Reproducibility

Not relevant for Coding issues. Otherwise `Always` or `Sometimes` — and **reproduction steps are
required**.

### Reason

| Reason | What to write in "Reason notes" |
|--------|--------------------------------|
| Lack of time | Briefly, how we would solve it given time |
| Lack of knowledge | Briefly, the difficulties we faced — including cases where fixing it created other problems we couldn't analyze in time |

## Likely ex3 candidates

Start the file early and add rows as we go. Things that plausibly end up here:

- Bonus items we deliberately skip: lazy load/unload of `.so` files, most of the optional rows in
  `docs/error-handling-matrix.md`, different-resolution map comparison.
- Known algorithm score ceilings carried from ex2 (e.g. the house-full floor layer below lidar `z_min`).
- Threading simplifications, e.g. locking where a lock-free design was possible.
- Any `-verbose` output we chose not to emit.
