---
name: populate-known-issues
description: Adds or updates rows in docs/known-issues.md using the course Known Issues.docx column schema. Use when documenting a skipped bonus, a known bug, a missing feature, a coding smell we will not fix before submission, or when preparing the Known Issues excel export.
---

# Populate Known Issues

Optional Known Issues list for TAU AdvCpp Assignment 3. Graders clone a Google Sheet, we fill it in
English, and we export `.xlsx` into the zip root at submission time. Until then the **only** working
file is markdown.

Source of truth, in this order:

1. `context/Known Issues.docx` (extract: `tmp/Known Issues.extracted.txt`)
2. **This skill**
3. `docs/known-issues-guidelines.md` (condensation — follow it when this skill is silent)

If this skill and the condensation disagree, the docx and this skill win. Do not invent columns,
types, reasons, or an in-repo `.xlsx`.

## When to invoke

Add or update a row when we are **shipping** one of these and will not fix it before the deadline:

- A skipped **bonus / optional** requirement (main expected use — see [Bonus and optional rows](#bonus-and-optional-rows)).
- A known **bug** (crash or misbehavior) we will not fix.
- A **missing or partial** required feature.
- A **coding smell** we will not clean up (duplication, too-long function, missing required class, …).
- Preparing the late Excel/Google-Sheet export from `docs/known-issues.md`.

### When NOT to invoke / NOT to list

Do **not** add a row for:

- Staff-skeleton conflicts (assignment vs published headers). Those live in `docs/open-questions.md`,
  not Known Issues.
- Open-question **working assumptions**, unless the Moodle Ex 3 forum later contradicted us and we
  still shipped the wrong behavior.
- Optional **"may"** items we chose not to do, **unless** graders could still deduct. Eager `.so` load
  is allowed; lazy load/unload is bonus — do not list eager load as a missing feature. Do list skipped
  lazy load (and skipped Common-issues-matrix optional rows) so graders see we tested and chose not to.
- Issues we just **fixed** — remove the row instead of leaving a stale bug.
- Invented issues, speculative "maybe later" items, or complaints about the skeleton.

## Working file

**Always** edit `docs/known-issues.md`.

That file is a markdown table of rows. The Excel / Google Sheet table is a **late submission export**.
Never create a `.xlsx` in the repo unless the user explicitly asks.

If `docs/known-issues.md` does not exist yet, create it with the header below and the first data rows.
Do not renumber existing `#` values when appending.

## Columns (match the docx example table)

```text
# | Type | Sub Type | Description | Severity | Reproducibility | Reproduce steps | Reason | Reason notes
```

Markdown form:

```markdown
| # | Type | Sub Type | Description | Severity | Reproducibility | Reproduce steps | Reason | Reason notes |
|---|------|----------|-------------|----------|-----------------|-----------------|--------|--------------|
```

Nine columns. No extras. No Hebrew. Short `Description`. Point at `file:line` in Description or
Reason notes when it helps a grader find the code.

## Type / Sub Type — use the EXAMPLE TABLE strings

The docx **prose** says "Feature Missing" and "Coding"; the **example table** uses Type `Feature` /
`Bug` / `Code` with a separate Sub Type cell (`Missing`, `Program misbehave`, …).

**Write the example-table strings in `docs/known-issues.md`** so the Excel export matches the staff
example. Do not mash Type and Sub Type into one cell (`Feature Missing`). Do not use Type `Coding`.

| Type | Sub Type (exact cell text) |
|------|----------------------------|
| `Feature` | `Missing` |
| `Feature` | `Partially Missing` |
| `Feature` | `developed in a different way` |
| `Bug` | `Program crash` |
| `Bug` | `Program misbehave` |
| `Code` | see coding subtypes below |

Prose meaning (docx): Feature = a requirement entirely or partially missing; Bug = bad runtime
behavior; Coding = imperfect code.

### Code subtypes (docx, verbatim intent)

Use one of these as the `Sub Type` cell when Type is `Code`:

| Sub Type | Notes |
|----------|--------|
| `Missing a required Class` | |
| `Having a redundant Class` | example table wording |
| `Code duplication` | |
| `Function is too long` | |
| `Class is doing a job of another class` | |
| `Spread logic` | logic of one operation spread into too many places |
| `Code is too complex` | could be written more elegantly |
| `Use of hard coded numbers instead of enums / constants` | **one row covers all cases** — do not document per number |
| `Bad names for functions / classes / variables` | **one row covers all cases** — do not document per name |
| `Other` | last resort |

## Severity

| Level | Meaning (docx) |
|-------|----------------|
| `High` | Submission is very poor — will not function in many cases, misses a major requirement, or the code is awful (e.g. everything in one class) |
| `Medium` | Will not function in some cases, misses a minor requirement, or the code is not good (e.g. a very long, complex function) |
| `Low` | Affects very rare cases, a small code issue, or a very minor missing requirement |

Skipped optional/bonus rows are usually `Low` or `Medium`, never `High` unless we also broke a
mandatory requirement.

## Reproducibility and Reproduce steps

| Kind | Reproducibility | Reproduce steps |
|------|-----------------|-----------------|
| Feature or Bug | `Always` or `Sometimes` | **Required.** Concrete steps a grader can follow. |
| Code | `Not relevant` | `---` |

`Always` = following the steps always shows the issue. `Sometimes` = following the steps only
sometimes shows it. Empty steps for a Feature/Bug row is invalid.

## Reason and Reason notes

`Reason` is **only** one of:

| Reason | Reason notes must say |
|--------|------------------------|
| `Lack of time` | Briefly **how we would solve it** given time |
| `Lack of knowledge` | Briefly the **difficulties** — including cases where a fix created other problems we could not analyze in time |

No other Reason values. Do not put the Reason sentence in Description.

## English only; grade effect

- Fill every cell in **English**. Questions: Moodle Ex 3 forum.
- The file is **optional**. Listing an issue is **grade-neutral-or-better** than the same issue
  undocumented (small items may cost nothing once listed). Staff will not deduct for something they
  did not plan to deduct for just because it appears here.
- **Omitting** something that should have been listed can cost **more** — it reads as "never tested."
  When in doubt on a real shipped gap, list it.

## How to add or update a row

1. Read `docs/known-issues.md`. Confirm the issue is not already there (same behavior, even if wording differs).
2. Confirm it is a real shipped gap — never invent issues.
3. Append a new row with `#` = max existing `#` + 1 (start at `1` if the table is empty).
4. Do **not** renumber other rows unless you are **deleting** a row (then compact `#` so they stay `1..n` with no holes).
5. Keep `Description` short. File:line in Description or Reason notes if it helps a grader.
6. If we **fixed** the issue, **delete that row** (then compact `#`). Do not leave a documented bug that no longer exists.
7. Do not edit AGENTS.md as part of this skill.

Example Feature row (skipped optional matrix item):

```markdown
| 1 | Feature | Missing | Faulty algorithm movement that would go out of bounds is not ignored by DroneControl | Low | Always | Run a plugin that commands a step past mission/world bounds; the movement is forwarded instead of dropped | Lack of time | Optional Common-issues-matrix row. Would ignore the command in DroneControlImpl before calling movement. See docs/error-handling-matrix.md. |
```

Example Code row:

```markdown
| 2 | Code | Function is too long | MappingAlgorithmImpl::nextCommand exceeds a readable length | Medium | Not relevant | --- | Lack of time | Split frontier selection from path follow-up in MappingAlgorithmImpl.cpp. |
```

## What must NOT go in the file

- Secrets, credentials, private keys, grader-hidden maps we shouldn't republish.
- Student-only process: workplan item codes (`s1`, `y9`, …), owner names as a namespace, meeting-point labels.
- "We didn't like the skeleton" / staff-API complaints.
- Things the assignment **explicitly allows** (eager `.so` load is allowed; lazy load is bonus — list
  only the skipped bonus, not the allowed eager path).
- Working assumptions from `docs/open-questions.md` unless the forum contradicted us.
- Mandatory matrix rows we actually implemented (do not list "we handle wall collisions" as an issue).

## Bonus and optional rows (main expected use)

Graders should see that we **tested** optional work and **chose not** to implement it. List each
skipped optional item as Type `Feature`, Sub Type `Missing` or `Partially Missing`, Severity `Low`
or `Medium`.

Walk these sources and add a row for every skip that will still be true at zip time:

1. **`docs/error-handling-matrix.md`** section **Optional (bonus)** — one row per skipped scenario
   (out-of-bounds ignore, invalid command retry, empty NOOP, empty LiDAR, movement `false`, split
   oversized moves, drone `Error` status, amend to stay in mission bounds, GPS OOB compare, GPS
   impossible after move). Implemented optional rows stay **off** the list.
2. **Lazy load/unload of `.so` files** — if we keep eager load (allowed), list lazy load as
   `Feature` / `Missing`.
3. Other assignment bonuses we skip (e.g. different-resolution map comparison, extra `-verbose`
   output we chose not to emit). Check `docs/known-issues-guidelines.md` "Likely ex3 candidates"
   and `docs/assignment3-checklist.md`.
4. Known algorithm limitations we will ship (e.g. ex2 house-full floor-layer ceiling) if still true.

Do not list a bonus we are claiming in `bonus.txt` as Missing.

## Before submission

1. Re-read `docs/known-issues.md` against the tree: delete fixed rows, add last-minute skips, keep English.
2. A human must **copy** the markdown table into the staff Google Sheet clone (docx: "This is a Google
   Sheet in the above structure that you may clone, fill and submit").
3. **Export Excel** (`.xlsx`) into the **zip root**, next to `students.txt` / `README.md`.
4. Do not put the `.xlsx` in the repo unless asked. The markdown file is the in-repo source.

Column reminder and grade rationale: `docs/known-issues-guidelines.md`.
Full staff text: `context/Known Issues.docx`.
