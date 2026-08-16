# Ex2 Grading Handoff — Lessons for Ex3

Source: instructor feedback for Ex2 (`Drone-Mapper-ex2/207190406_209543255_grading/feedback.txt`),
the bug index (`ex2_bugs.xlsx`), the grading methodology doc (`Exercise 2 - Grading Explanation.docx`),
and a diff of the mutated grading sources against our submitted `src/`. Final score: **77.5/100**.

This is not just a "what to fix" list for ex2's code (that project is done) — it's a list of *patterns*
that cost us points, so we don't reproduce them in Ex3 with different class/function names.

## Where this fits in priority for Ex3

This doc should **not** be the main driver of how we plan or sequence Ex3 work — the Ex3 spec (new
Simulator, threading, dynamic `.so` loading, comparative/competitive modes) is the primary source of
truth, and getting the required functionality built and correct comes first.

That said, treat the lessons here — especially "write solid component + integration tests, including
for edge/error cases" — as a **safe default requirement for good code**, not an optional extra. As of
this writing, the Ex3 assignment doc (`context/Advanced Topics TAU 2026B - Assignment 3.docx`, still in
draft) has zero mentions of tests/testing/coverage, unlike Ex2's spec which had a dedicated section with
explicit coverage targets. But Ex2's grading showed that untested requirements and edge cases get
penalized as if they were bugs regardless of whether the spec spells out a testing section. So: build the
new Simulator/threading/dynamic-loading code with the same testing discipline as before by default, and
revisit if/when the draft is updated with explicit testing requirements — don't treat the current silence
as permission to skip it.

## How this kind of grading works (applies to Ex3 too)

The instructor doesn't read every line by hand. They compile our code **28 different times**, each time
with one specific bug silently injected (behavior changed slightly, e.g. a comparison flipped or a check
removed), then run **our own test suites** (component + integration) against each mutant and see whether
we notice. Our grade is basically: *how much of your own implicit spec did your own tests actually enforce?*

Implications for Ex3:

- **Every requirement in the spec is a potential mutation target.** If the spec says "X must happen when Y,"
  assume there's a test injecting "X doesn't happen when Y" and check whether *our* tests would fail.
- **A missing feature and a missing test look identical to the grader** in the worst case: if the behavior
  isn't implemented, there's nothing for the bug to break, and it's marked "obsolete" — which was
  penalized as heavily as an uncaught bug. Implementing a partial/wrong version of a required check is
  actually *safer than not implementing it at all*, because at least it gives the mutation something to
  break, which our tests can then catch.
- **A test suite that hangs or crashes on unexpected input is its own liability**, independent of whether
  the underlying feature is correct (see "Test robustness" below).

## Category-by-category breakdown

### 1. Obsolete bugs (missing functionality) — cost us 7 pts

Two bugs had nothing to inject because the corresponding functionality didn't exist in our code:

- A YAML/summary-output feature the bug was supposed to corrupt was never implemented, so there was
  no code path to mutate.
- A null/defensive check on an input collection (comparing against a null/empty map) was expected to
  exist somewhere in a comparison utility; because we never added that guard, the "corrupt this check"
  bug had nothing to attach to.

**Lesson for Ex3:** Treat every "handle this edge case" and "what if X is missing/null/invalid" line in
the spec as a hard requirement, not a nice-to-have — even if it feels like it'll never happen in practice.
Missing edge-case handling is penalized like a real bug, not like a smaller issue. When in doubt, add the
defensive check/branch even if we can't fully test every path.

### 2. Interface/API changes — cost us 5 pts

We deviated from the staff-provided API/types in at least one place. `feedback.txt` doesn't say which
one, so we diffed our headers under `Drone-Mapper-ex2/include/` against the staff skeleton in
`ex_2_skeleton/include/drone_mapper/` to find it. Two real deviations turned up:

- **Most likely culprit — a renamed struct field in a staff-owned type.** In
  `types/MissionTypes.h`, the staff's `MissionConfigData` struct has a field named `mission_bounds`.
  Our submission had the same field, same purpose, but named it `boundaries` instead:

  ```cpp
  // staff skeleton (ex_2_skeleton)
  struct MissionConfigData {
      ...
      // 20.6 - readded the map boundaries to the MissionConfig for outputting
      MappingBounds mission_bounds{};
  };

  // our submission (Drone-Mapper-ex2)
  struct MissionConfigData {
      ...
      MappingBounds boundaries{};
      std::optional<ErrorRef> config_load_error{};
  };
  ```

  Root cause, traced through git history on both sides: the staff skeleton's own field name churned
  during the semester — it was originally `boundaries`, got removed, then was **re-added as
  `mission_bounds`** in a mid-course skeleton update. We added this field to our code shortly *after*
  that staff update, but used the old/pre-update name (`boundaries`) instead of picking up the current
  skeleton's name (`mission_bounds`). In other words: we built against a stale mental model of the
  skeleton instead of re-checking the current published version at the time we wrote the code. The field
  still compiled and worked fine against our own code (which is exactly why this kind of drift is easy to
  miss) — it only matters where staff code or staff tests construct/read that struct by field name.

- **Runner-up, less likely to be the specific flagged case, but a real deviation nonetheless —
  `SimulationCompositionData` reshaped.** In `types/SimulationTypes.h`, the staff version nests
  simulation/mission data as `std::vector<std::tuple<SimulationConfigData, std::vector<MissionConfigData>>>
  simulation_mission_groups`. Our version flattened this into two separate parallel vectors
  (`simulations` and `missions`) instead. This is a bigger structural change than the field rename above,
  which is exactly why it's the less likely candidate for a single "-5 pts, count=1" penalty (it reads
  more like a deliberate redesign than an accidental one-line drift) — but it's still a real, uncorrected
  shape deviation from the staff type that should be reconciled regardless of which one triggered the
  actual penalty.

**Lesson for Ex3:** Never modify the signature, semantics, field names, or shape of a staff-provided
interface/type to make our own implementation more convenient — even a same-purpose field with a
different name counts as a deviation, not just a changed method signature. If something about the given
API seems awkward or wrong, raise it on the course forum rather than quietly changing it — a silent
interface change is graded as a correctness violation, not a style choice, and it's a flat penalty per
instance (uncapped by "just do it once" logic).

Concretely, since the Ex2 root cause was **building against a stale/remembered version of the skeleton
instead of the current published one**, and Ex3's spec explicitly states interfaces are unchanged from
Ex2:
- Before writing code against any staff-provided type (in `common/`, `UserCommon/` if shared, or any
  published header), re-pull/re-check the **current** version of that header rather than trusting memory
  or an older local copy — the staff skeleton can and does change mid-course.
- Periodically (e.g. before each milestone/PR, and definitely before final submission) **diff our copy of
  every staff-provided header/type against the latest published skeleton**, not just the pure abstract
  interfaces (`I*.h`) — struct/field names in shared data types are just as much "the interface" as
  virtual method signatures, and a renamed field still compiles cleanly against our own code, so nothing
  will warn us locally if it drifts.
- If we ever need to reshape a staff-provided struct (like the `SimulationCompositionData` case) because
  the given shape seems inconvenient, that's a signal to ask on the forum first, not to just adapt it
  silently — and if we do adapt a local copy for some reason, keep an explicit TODO/reminder to reconcile
  it back before submission.

### 3. Timeouts — capped penalty, but a symptom of a deeper problem

Our component test suite **timed out on 27 of 28 injected bugs** (effectively every mutation we tested).
Because the formula caps this penalty (`min(5, timeouts)`), it cost us the same fixed amount whether it
was 1 timeout or 27 — but that's a lucky break, not something to rely on. A near-universal timeout pattern
almost certainly means:

- A test or fixture doesn't terminate deterministically when the code under test behaves slightly
  differently than expected (e.g. a loop bounded by a sensor/measurement condition that a mutated
  component no longer satisfies, so it spins forever instead of returning/erroring).
- Tests may be relying on real timing/sleeps/threads instead of injectable/mockable clocks or bounded
  iteration counts.

**Lesson for Ex3:**
- Any loop that depends on "wait until condition C from a dependency/mock" must have a hard iteration
  cap or timeout guard, so a misbehaving dependency produces a **failing assertion**, not a hang.
- Prefer deterministic mocks/fakes over real timing wherever a test's correctness depends on some
  external condition eventually becoming true.
- Add timeouts at the test-framework level (e.g. per-test timeout) as a safety net so a single bad
  interaction fails fast and loud instead of hanging the whole suite.
- Before submitting, deliberately try "breaking" a few of our own components (flip a condition, skip a
  step) and confirm the tests **fail with a clear message**, not hang.

### 4. Coverage targets — landed almost exactly on target, don't assume we'll be this lucky again

Component-test coverage landed at exactly the 50% target and integration coverage exceeded the 25%
target (which is a bonus). This looks better than it may actually be — the target isn't "hit the minimum
and stop," it's a floor, and exceeding it is straightforwardly rewarded with no cap on the bonus side.

**Lesson for Ex3:**
- Don't aim for "just enough" coverage. Since overshooting the coverage target is a pure bonus with no
  visible downside, write component/integration tests generously, especially around any conditional
  logic, boundary values (min/max, zero/negative, off-by-one), and error-status handling.
- Structure tests so each one asserts one specific behavior/invariant — broad "smoke tests" that only
  check "the program didn't crash" won't catch a subtly flipped condition or swapped priority ranking,
  which is exactly the kind of mutation the grader injects.
- Explicitly test error/edge statuses (e.g. an "error" or "invalid" result type) getting propagated,
  persisted, or reported correctly — several of our ex2 bugs specifically targeted "does the error/status
  path behave differently than the success path" (e.g. skipping a save, dropping a result from an
  aggregate, or scoring an errored run) and those are cheap, high-value tests to write.

## General process lessons (not code-specific)

1. **Read grading/exception rules as carefully as the spec itself.** Ex2's grading doc listed a handful
   of explicit exceptions (certain bugs allowed to time out, certain missing behaviors not counted as
   "obsolete," one bug expected to intentionally crash a specific suite). If Ex3 publishes similar
   grading/exception notes, they materially change which edge cases are worth prioritizing — read them
   before finalizing what "done" means.
2. **Our own tests are graded as our specification.** Since bugs are only "covered" if *our* tests fail,
   writing thin or duplicate tests (we noticed at least one bug ID double-counted in our own coverage
   list, suggesting a duplicate/overlapping test) doesn't help — coverage is about distinct behaviors
   caught, not test count.
3. **Defensive checks pay for themselves twice.** They both (a) satisfy "obsolete bug" requirements by
   giving the grader's mutation something to break, and (b) are usually one-line, high-value tests to add
   ourselves (null checks, out-of-range checks, empty-collection checks).
4. **Before submission, do our own mini mutation pass.** Pick a handful of the riskiest conditionals
   (direction flips, off-by-one bounds, min/max swaps, priority/ranking comparisons, status-dependent
   branches) in the actual code, invert them locally, and confirm a test fails. This mirrors exactly what
   the grader does and is the cheapest way to estimate our real coverage before submitting.
5. **Don't let a passing build hide a hanging test.** Always run the full test suite with a wall-clock
   timeout locally (e.g. via CI or a test-runner flag) so a hang is caught before submission, not
   discovered in the grading report.
