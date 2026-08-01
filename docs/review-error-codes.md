# Review error codes

From `context/Error Code Key.xlsx` (unchanged since ex1/ex2). Severity columns: minor (m), normal (n),
severe (s) — the number is points deducted.

No assignment-3-specific review guideline has been published. Until one is, assume the AdvCpp rubric
below plus `context/AdvCpp Review Guideline.docx` still govern manual review.

## Not the same as runtime `ErrorRef::code`

These rubric codes (`e01`–`e23`, `b01`–`b08`) grade **submission quality**. They never appear in program
output. Runtime semantic codes (e.g. `MAP_FILE_NOT_FOUND`, `DRONE_HITS_OBSTACLE`) live in
`common::types::ErrorRef::code` and go into error logs and report YAML.

| Concern | Rubric code | Runtime code |
|---------|-------------|--------------|
| Handler takes a free-form string instead of structured info | `e05` | — |
| Scenario fails without proper handling | `b04` | `score: -1` + `ErrorRef` |
| Missing input file | `b06` | e.g. `MAP_FILE_NOT_FOUND` |

## Code review (e*)

| Code | Description | m | n | s |
|------|-------------|---|---|---|
| e01 | Unrelated grouped classes in header | 0 | 1 | 2 |
| e02 | Too granular header separation | 0 | 0 | 1 |
| e03 | Unwrapping mp-units types for math | 0 | 0 | 2 |
| e04 | Incorrect use of C++ stdlib | 0 | 1 | 3 |
| e05 | Passing diagnostic message into handler | 0 | 1 | 1 |
| e06 | const-ref params passed as mutable-ref/copy | 0 | 1 | 2 |
| e07 | Class methods could be const | 0 | 1 | 2 |
| e08 | Unneeded methods / data in class API | 0 | 1 | 2 |
| e09 | Too long functions | 0 | 1 | 2 |
| e10 | Duplicate class, functions, or flows | 0 | 1 | 3 |
| e11 | Library requires manual install | 1 | 2 | 4 |
| e13 | Incorrect use of pointers | 0 | 1 | 2 |
| e14 | Incomplete class diagram in HLD | 0 | 1 | 3 |
| e15 | Incomplete sequence diagram in HLD | 0 | 1 | 3 |
| e16 | Numerical fields instead of strong types | 0 | 1 | 2 |
| e17 | Unnecessary dependency | 0 | 1 | 3 |
| e18 | Noncompliant submission | 0 | 2 | 4 |
| e21 | Repeated computation | 0 | 1 | 3 |
| e22 | Poor encapsulation | 1 | 1 | 2 |
| e23 | Magic numbers | 0 | 1 | 1 |

`e18` is the one to watch in ex3: wrong `.so` name, wrong namespace, missing `UserCommon/`, files added
to `common/`, or a missing build file are all "noncompliant submission".

## Program run (b*)

| Code | Description | m | n | s |
|------|-------------|---|---|---|
| b01 | Build failure of main | 1 | 3 | 5 |
| b02 | Build failure of extra | 1 | 2 | 3 |
| b03 | Missing scenario validation | 1 | 1 | 1 |
| b04 | Error on scenario | 3 | 10 | 30 |
| b05 | Timeout on scenario (1 min) | 2 | 5 | 20 |
| b06 | No error on missing input | 1 | 1 | 1 |
| b07 | Manual config change failure | 5 | 10 | 10 |
| b08 | Scenario required manual change | 5 | 5 | 5 |

## AdvCpp review guideline summary

From `context/AdvCpp Review Guideline.docx`:

- Happy flow completes on all default scenarios, including **from a different path** and after **minor
  valid config edits**.
- Invalid config / input → exit gracefully.
- Reasonable runtime for all cases.
- Coherent structure, clear separation of responsibilities, acyclic `#include` DAG, no unnecessary includes.
- const-correctness, maximal encapsulation, lean classes; helpers that don't need class state go to an
  anonymous namespace in the `.cpp`.
- No `new`/`delete` where avoidable, never `malloc`/`free`; rule of 3/5 for classes owning memory.
- No magic numbers, no expensive recomputation, correct ref/const-ref usage, correct std containers and
  smart pointers, short single-responsibility functions.
- HLD and readme must match the code.
- `bonus.txt` if applicable.
