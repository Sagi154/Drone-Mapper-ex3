# Open questions

Genuine ambiguities found while reading the assignment 3 doc, the skeleton, and the supporting context.
Each has a **working assumption** so work is not blocked; raise the ones that matter on the Moodle Ex 3
forum and update this file with the answer.

Agents: follow the working assumption. If the code contradicts one of these, fix this file too.

---

## 1. Namespace naming conflicts with the skeleton README

The assignment requires all our code to sit in ID-suffixed namespaces (`Algorithm_<ids>`,
`MissionControl_<ids>`, `UserCommon_<ids>`). The skeleton `README.md` says:

> Use the lowercase project namespaces `common`, `algorithm`, `mission_control`, and `simulator` in your
> implementation.

These cannot both be literally true. `common` must stay `common` regardless — the registration macros
hard-code `::common::MappingAlgorithmRegistration` and `::common::MissionControlRegistration`.

**Working assumption:** follow the assignment doc for our own code (`namespace
Algorithm_207190406_209543255 { ... }`), since ID-suffixed namespaces are what actually prevents symbol
collisions when two teams' `.so` files are loaded into the same process. Treat the README's lowercase
names as describing the *published interfaces*.

**Ask:** should our implementation namespaces be nested (`algorithm::Algorithm_<ids>`) or flat
(`Algorithm_<ids>`)?

---

## 2. `UserCommon/` is missing from the skeleton

The assignment lists 5 folders and requires `UserCommon/`; `ex_3_skeleton` ships only 4
(`Simulator`, `Algorithm`, `MissionControl`, `common`).

Relatedly, the skeleton puts published interfaces in **project-local** `common_simulator/` and
`common_mission_control/` subfolders, which the assignment's 5-folder description doesn't mention.

**Working assumption:** create `UserCommon/` ourselves (no build file — each project compiles the sources
it needs, per the assignment). Keep the skeleton's `common_*` subfolders exactly where they are; they are
part of the `Simulator`/`MissionControl` folders, so the submission still has 5 top-level folders.

---

## 3. Scoring: how are `total_score` and `total_steps` defined?

The report formats show `total_score` and `total_steps` per mission control / per algorithm, but the
assignment never defines them. Ex2's `MapsComparison` is not in the ex3 skeleton at all.

**Working assumption:** port ex2's `MapsComparison` into `Simulator/src/`, keep its 0–100 per-run score
and `-1` on failure, and define `total_score` as the **sum of per-run scores** and `total_steps` as the
**sum of `MissionRunResult::steps`** across all runs in the composition for that plugin. The example
values in the doc (495, 502, 100, 124) are consistent with sums over multiple runs.

**Ask:** is the score metric still ex2 map comparison, and are `total_score` / `total_steps` sums?

---

## 4. Comparative mode: what makes two mission controls "the same result"?

`results_summary` groups mission controls whose results are identical (`same_results: [...]`), but the
equality relation is unspecified. Candidates: identical output maps, or identical `(total_score,
total_steps)` — note the doc's own example has group 1 and group 3 both at `total_score: 495` but
different `total_steps` (100 vs 101), and they are *not* grouped.

**Working assumption:** group by the exact `(total_score, total_steps)` pair, since that is what the
example is consistent with and it is what the report displays. Sort groups by group size descending
("number of agreeing managers").

**Ask:** is grouping by score+steps, or by byte-identical output maps?

---

## 5. Are `.cw` map files an input format?

`inputs/map/` adds `npy_to_cw.py`, `scenario_small.cw`, `scenario_big.cw`, `benchmark_map.cw` — gzipped
NBT ClassicWorld files. Every `inputs/simulation/*.yaml` still points at a `.npy`, and `tinynpy` is still
in `vcpkg.json`.

**Working assumption:** `.cw` is a visualization aid for ClassiCube. Read only `.npy`; do not add an NBT
dependency.

---

## 6. Are tests still graded, and how?

Assignment 3 says nothing about tests, bug injection, or component test suites — unlike ex2, whose whole
grade hinged on them. `vcpkg.json` still lists `gtest`, and the devcontainer installs `libgtest-dev`.
No Ex 3 Review Guideline has been published.

**Working assumption:** keep and port the ex2 test suites (they protect the port itself), but do not
invest in new bug-injection coverage until a review guideline appears. Keep the ex2 suite filter names so
nothing is lost if bug injection returns.

**Ask:** will ex3 be graded by bug injection like ex2? Which test filter names, if so?

---

## 7. Does `-verbose` reach the Algorithm?

`MissionControlDependencies` has a `verbose` flag; `MappingAlgorithmDependencies` does not. The
assignment says "The MissionControl shall create output files with verbose info … iff `-verbose`", and
separately that "The MissionControl and Algorithm project may create error logs".

**Working assumption:** only `MissionControl` gets `-verbose`. The algorithm may write an error log
unconditionally but no verbose output, since it has no way to learn the flag.

---

## 8. Unresolved `[TBD]` in the assignment doc

The doc's final "Additional Notes and Requirements" bullet is literally `[TBD]`. The doc left draft mode
on Jul 26, 2026, and post-draft changes are supposed to be highlighted.

**Action:** re-extract and diff the docx before submission. Also check the Ex 3 forum for changes that
create extra work — the doc explicitly says to raise those on the forum rather than rushing to rewrite code.
