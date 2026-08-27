# Assignment 3 Simulator — Black-box Test Catalog

**Status of the spec:** Assignment 3 was in DRAFT mode (“till July-26th 23:00”) and its last Additional Notes item is literally `[TBD]`. YAML examples end with a standalone `[...]`. This catalog does not invent the missing definitions.

**What a submission is:** unzip `ex3_<ids>.zip` → 5 folders (`Simulator/`, `Algorithm/`, `MissionControl/`, `common/` as-is, `UserCommon/`), 4 build files (root + three project folders), `students.txt`, `README.md`. Build with **that team’s** tooling. Artifacts: `simulator_<ids>`, `Algorithm_<ids>.so`, `MissionControl_<ids>.so`. The binary `dlopen`s whatever the CLI points at (`<algorithm_so>`, `<mission_control_so>`, or a folder of such files).

**Placeholders (never concrete IDs):** `<ids>`, `<algorithm_so>`, `<mission_control_so>`. Doc-original tokens (`<submitter_ids>`, `<algorithm_so_filename>`, …) appear only inside quotes.

**Source priority:** frozen headers + this skeleton’s root `CMakeLists.txt` / `CMakePresets.json` / `README.md` win on anything they actually define. Assignment 3 docx governs CLI, reports, threading, zip, errors. Common-issues PDF governs mandatory vs optional faults. Structuring PDF governs mock/registration placement. Assignment 1/2 and the older Submission Guidelines are **not** merged.

**Build discovery (every runtime test assumes this):** a generic tester must find and invoke whatever the submission provides (root `CMakeLists.txt` and/or `Makefile` and/or other). The skeleton’s `cmake --preset default` is **not** guaranteed to still be there. The only portable post-build names are `simulator_<ids>`, `Algorithm_<ids>.so`, `MissionControl_<ids>.so`.

**Sources used:** frozen headers + root/project CMake + `CMakePresets.json` + `README.md` + `inputs/*.yaml`; Assignment 3 `document.xml`; Common issues PDF; Structuring the project PDF. **Flagged, not merged:** Submission Guidelines (old `drone_mapper`); AdvCpp Review + Error Code Key (manual / `b*` harness hints); Known Issues process template. **Ignored:** Assignment 1/2 docx, Exercise 2 Review Guideline.

---

## Mandatory CLI/runtime behaviors

### CLI-01 — Comparative invocation

- **Quote (docx, “The Simulator”):** `./simulator_<submitter_ids> -comparative simulation=<simulation_composition_yaml_filename> mission_control_folder=<mission_control_folder> algorithm=<algorithm_so_filename> [num_threads=<num>] [-verbose]`
- **Setup:** After a successful team build, with a readable composition YAML (shape as in `inputs/sim_compose.yaml`), a traversable folder containing ≥1 loadable MissionControl `.so`, and one loadable `<algorithm_so>`:

```text
./simulator_<ids> -comparative simulation=<composition.yaml> mission_control_folder=<mc_folder> algorithm=<algorithm_so>
```

- **Expected:** Process does not crash. Creates `<mc_folder>/comparative_results_<time>/` and writes the comparative artifacts below. Arguments may appear in any order. `key=value` has no spaces around `=`.
- **Classification:** MANDATORY

### CLI-02 — Competition invocation

- **Quote (docx):** `./simulator_<submitter_ids> -competition simulation=<simulation_composition_yaml_filename> mission_control=<mission_control_so_filename> algorithms_folder=<algorithms_folder> [num_threads=<num>] [-verbose]`
- **Setup:**

```text
./simulator_<ids> -competition simulation=<composition.yaml> mission_control=<mission_control_so> algorithms_folder=<alg_folder>
```

- **Expected:** Process does not crash. Creates `<alg_folder>/competition_<time>/` (name is `competition_<time>`, **not** `competition_results_<time>`).
- **Classification:** MANDATORY

### CLI-03 — Argument order

- **Quote (docx):** “All command line arguments can appear in any order.”
- **Setup:** Same as CLI-01/CLI-02 with permuted tokens (`-comparative`, `simulation=…`, `algorithm=…`, `mission_control_folder=…`).
- **Expected:** Same acceptance and same class of outputs as canonical order.
- **Classification:** MANDATORY

### CLI-04 — Missing required arguments → usage, then finish

- **Quote (docx):** “In case command line arguments are missing, the program should print a usage with an error message detailing the missing command lines arguments, then finish.”
- **Setup:** e.g. `./simulator_<ids> -comparative` with no `simulation=`, and separately omit `mission_control_folder=` / `algorithm=` (and the competition analogs `mission_control=` / `algorithms_folder=`).
- **Expected:** Some usage text is printed; the error mentions the missing argument **name(s)** (`simulation`, `mission_control_folder`, `algorithm`, `mission_control`, `algorithms_folder` as applicable); then the process ends. Exact wording is “for your decision.” Exit code and stdout vs stderr are unspecified — only “print” / “finish.”
- **Classification:** MANDATORY (wording OPTIONAL). Whether omitted `num_threads`/`-verbose` count as “missing” is **ambiguous** (see contradictions).

### CLI-05 — Unsupported arguments → usage naming all of them, then finish

- **Quote (docx):** “If unsupported command lines arguments are provided, the program should print a usage with an error message pointing at **all** the unsupported command lines arguments provided, then finish.”
- **Setup:** Otherwise-valid CLI-01 plus an extra token and/or unknown `foo=bar` (and a second unsupported token to check “all”).
- **Expected:** Usage printed; error mentions **every** unsupported token; then finish. Wording “for your decision.”
- **Classification:** MANDATORY (wording OPTIONAL)

### CLI-06 — File argument missing or unreadable → usage, then finish

- **Quote (docx):** “In case a command line argument that should point at a file is pointing at a non-existing file or one that cannot be opened, the program should print a usage with a proper error message, then finish.”
- **Setup:** `simulation=` → nonexistent path; `algorithm=` / `mission_control=` → nonexistent `.so`.
- **Expected:** Usage + some error; finish. Wording “for your decision.”
- **Classification:** MANDATORY

### CLI-07 — Folder missing, unreadable, or zero desired files → usage, then finish

- **Quote (docx):** “In case a command line argument that should point at a folder is pointing at a non-existing folder or one that cannot be traversed or at a folder that has **zero files of the desired usage** the program should print a usage with a proper error message, then finish.”
- **Setup:** `mission_control_folder=` / `algorithms_folder=` (1) does not exist, (2) cannot be traversed, (3) exists but contains no relevant `.so` files (empty, or only non-plugin files).
- **Expected:** Usage + some error; finish. Glob for “desired usage” is not named — implied MissionControl vs Algorithm `.so`s.
- **Classification:** MANDATORY (exact file-type filter **ambiguous**)

### CLI-08 — Cannot create results directory → error to screen

- **Quote (docx):** “In case the folder cannot be created, write a proper error to screen.” (stated for both modes)
- **Setup:** Make `<mc_folder>` / `<alg_folder>` non-writable so `comparative_results_<time>` / `competition_<time>` cannot be created.
- **Expected:** An error is written to “screen.” This sentence does **not** require a usage dump. Wording “for your decision.” Exit code unspecified.
- **Classification:** MANDATORY

### PLUGIN-01 — Comparative work: all MissionControls × all composition configs × given algorithm

- **Quote (docx):** “Should run all MissionControl implementations in the given folder, on all the configurations in the simulation yaml config, and with the given algorithm.”
- **Setup:** CLI-01; folder with N distinct loadable `<mission_control_so>` files; composition YAML with G simulation–mission groups (and composition-level drone/lidar lists — see YAML-IN-01).
- **Expected:** N MissionControls are exercised with the given `<algorithm_so>` across the composition. Per-MC result YAMLs appear (YAML-OUT-03). Exact cartesian product of drones × lidars × missions is **not defined** in prose — see UNSPEC-COMBINE.
- **Classification:** MANDATORY (combination details ambiguous)

### PLUGIN-02 — Competition work: given MissionControl × all algorithms

- **Quote (docx):** “Should run the given MissionControl using all algorithms.”
- **Setup:** CLI-02; folder with K loadable `<algorithm_so>` files.
- **Expected:** Given `<mission_control_so>` is run with every algorithm in the folder. Per-algorithm result YAMLs appear. Whether “all the configurations in the simulation yaml config” also applies in competition is **only stated under comparative** — **ambiguous**.
- **Classification:** MANDATORY (composition-crossing in competition ambiguous)

### PLUGIN-03 — Dynamic `.so` loading from CLI paths

- **Quote (docx):** “Algorithms and the MissionControl(s) are loaded dynamically as `.so` files.” Skeleton `Simulator/CMakeLists.txt`: “export the registration-constructor symbols needed by dynamically loaded plugins.”
- **Setup:** Place grader-built mock plugins (using frozen `REGISTER_MAPPING_ALGORITHM` / `REGISTER_MISSION_CONTROL`) at paths **other than** the submission’s own artifact directory; point CLI at those paths/folders.
- **Expected:** Those plugins are the ones run (summary YAML lists their filenames). Loading is from “wherever the CLI points,” not a hardcoded name.
- **Classification:** MANDATORY

### PLUGIN-04 — Unloadable `.so` listed under YAML `errors`

- **Quote (docx, comparative example comment):** `errors: ["manager7.so", "manager8.so"] # could not be loaded / run` (competition analog uses algorithm filenames).
- **Setup:** Folder contains at least one loadable plugin (so CLI-07 does not fire) **and** one file that cannot be `dlopen`ed / has no registration (e.g. empty `.so` or a text file named like a plugin if the loader tries it — if the loader ignores non-`.so`, use a corrupt `.so`).
- **Expected:** That plugin’s filename appears in the summary YAML `errors` list. Simulator does not crash. Whether “could not be … run” includes mid-run failure is **ambiguous**; plugin **crash** need not be handled (ERR-01).
- **Classification:** MANDATORY for load failure

### OUT-01 — Comparative results directory

- **Quote (docx):** “Should create an output directory directly under the `mission_control_folder` provided, with the name `comparative_results_<time>`.” “For the `<time>` part use code that will generate a new number per time to avoid collision with existing files if any.”
- **Setup:** Successful CLI-01.
- **Expected:** New directory `<mission_control_folder>/comparative_results_<time>/`. Not cwd and not beside the composition YAML unless those paths coincide. `<time>` is “a new number”; exact format is student-decided. Two successive runs must not clobber by reusing the same folder name if the first still exists.
- **Classification:** MANDATORY (time format OPTIONAL)

### OUT-02 — Competition results directory

- **Quote (docx):** “Should create an output directory directly under the `algorithms_folder` provided, with the name `competition_<time>`.”
- **Setup:** Successful CLI-02.
- **Expected:** `<algorithms_folder>/competition_<time>/`.
- **Classification:** MANDATORY

### OUT-03 — Contents of the results folder (both modes)

- **Quote (docx):** results folder contains “All output map files, with a unique name per each file (you should decide on the unique name pattern, but should make it easy to relate each file to the mission that created it).” “Error log(s) files.” “A single Simulation Result Output File - as a YAML file.” Plus per-plugin Assignment-2 YAMLs “added to the same output folder.”
- **Setup:** Successful CLI-01 or CLI-02 that actually runs ≥1 mission.
- **Expected (presence only):**
  1. Unique output map files in that folder; names relatable to the mission (pattern “for your decision”).
  2. One or more error-log files (names/format unspecified).
  3. Exactly one summary YAML (`comparative_report` or `competitive_report`).
  4. Additional per-plugin YAMLs (YAML-OUT-03).
- **Classification:** MANDATORY presence; names/formats partly OPTIONAL / unspecified

### YAML-OUT-01 — Comparative summary YAML structure and sort

- **Quote (docx):**

```yaml
comparative_report:
  composition_file: "simulation_compositions.yaml"
  mission_control_folder: "folder"
  generated_at_utc: "2026-05-30T23:31:10Z"

  results_summary: # sorted by number of agreeing managers, descending
    - same_results: ["manager1.so", "manager2.so", "manager5.so"]
      total_score: 495
      total_steps: 100

  errors: ["manager7.so", "manager8.so"] # could not be loaded / run
```

- **Setup:** CLI-01; inspect the single summary YAML under `comparative_results_<time>/`.
- **Expected:** Top key `comparative_report`. Child keys (quoted): `composition_file`, `mission_control_folder`, `generated_at_utc`, `results_summary`, `errors`. Each `results_summary` element has `same_results` (list of `.so` filenames), `total_score`, `total_steps`. `results_summary` is sorted **descending by number of agreeing managers** (`len(same_results)`). Tie-break unspecified. `composition_file` should identify the composition that was run. Filename of this summary file is **unspecified**. Trailing `[...]` in the doc — extra keys not forbidden, not required.
- **Classification:** MANDATORY structure + sort; equality metric for `same_results` is GENUINELY UNSPECIFIED (test grouping only after an external definition, or only test list-of-strings shape)

### YAML-OUT-02 — Competitive summary YAML structure and sort

- **Quote (docx):**

```yaml
competitive_report:
  composition_file: "simulation_compositions.yaml"
  mission_control: "mission_control_filename.so"
  generated_at_utc: "2026-05-30T23:31:10Z"

  results_summary: # sorted by score descending, then by steps ascending
    - algorithm: "algorithm1.so"
      total_score: 495
      total_steps: 100

  errors: ["algorithm2.so", "algorithm5.so"]
```

- **Setup:** CLI-02; inspect summary YAML under `competition_<time>/`.
- **Expected:** Top key `competitive_report`. Child keys: `composition_file`, `mission_control`, `generated_at_utc`, `results_summary`, `errors`. **No** `algorithms_folder` key in the example — do not require it. Each summary row: `algorithm`, `total_score`, `total_steps`. Sort: **score descending, then steps ascending**. Further ties unspecified. `mission_control` should be the given `<mission_control_so>` filename.
- **Classification:** MANDATORY structure + sort; score formula GENUINELY UNSPECIFIED

### YAML-OUT-03 — Per-plugin “assignment 2” YAML in the same folder

- **Quote (docx):** Comparative: “all runs, per each mission control, will generate their own Simulation Result Output File - YAML as in assignment 2. With the name of the mission control added to the file name.” Competitive: same “per each algorithm” / “name of the algorithm added.”
- **Setup:** CLI-01 with 2 loadable MissionControls; CLI-02 with 2 loadable Algorithms.
- **Expected:** Additional YAML files in the **same** results folder; each successful plugin has a file whose name includes that plugin’s name. Schema is “as in assignment 2,” which this catalog was instructed **not** to merge — **do not assert Assignment-2 key names**. Skeleton has **no** sample output YAML (only `.gitignore` hint `simulation_output.yaml`, which is **not** the comparative filename pattern).
- **Classification:** MANDATORY that the files exist, sit in the results dir, and incorporate the plugin name; schema GENUINELY UNSPECIFIED here

### YAML-IN-01 — Accept composition YAML with skeleton key tree

- **Quote (skeleton `inputs/sim_compose.yaml` — highest priority for input shape; docx only says `simulation=` is a YAML file):**

```yaml
simulation_compositions:
  simulations:
    - simulation_config: <path>
      mission_configs:
        - <path>
  drone_configs:
    - <path>
  lidar_configs:
    - <path>
```

Keys quoted: `simulation_compositions`, `simulations`, `simulation_config`, `mission_configs`, `drone_configs`, `lidar_configs`. `drone_configs` / `lidar_configs` are **siblings of `simulations`**, not nested per simulation (this matches `simulator::types::SimulationCompositionData`).

Nested file shapes (skeleton samples):

| Kind | Quoted keys |
|---|---|
| drone | `drone_config.dimensions_cm`, `max_rotate_deg`, `max_advance_cm`, `max_elevate_cm` |
| lidar | `lidar_config.z_min_cm`, `z_max_cm`, `d_cm`, `fov_circles` |
| mission | `mission_config.max_steps`, `boundaries.x_boundary.{min_cm,max_cm}`, `y_boundary.{min_cm,max_cm}`, `height_boundary.{min_cm,max_cm}`, `gps_resolution_cm` |
| simulation | `simulation_config.map_filename`, `map_resolution_cm`, `initial_drone_position.{x_cm,y_cm,height_cm}`, `initial_angle_deg`, `map_axes_offset.{x_offset,y_offset,height_offset}` |

Simulation comment: `# 0=east, 90=south, 180=west, 270=north`. Maps referenced as `map/*.npy`.

- **Setup:** CLI-01/02 with a composition cloned from `inputs/sim_compose.yaml` (paths adjusted).
- **Expected:** Files open; run proceeds. Invalid YAML content: **not specified** (ERR-YAML / UNSPEC-13).
- **Classification:** MANDATORY as the skeleton’s input contract

### LOG-01 — `-verbose` iff MissionControl verbose files

- **Quote (docx):** “The MissionControl shall create output files with verbose info (anything you think is of interest), iff `-verbose` appears on the command line.” Header: `MissionControlDependencies.verbose = false`.
- **Setup:** Pair of otherwise identical runs, with and without `-verbose`.
- **Expected:** Verbose MC output files appear **only** when `-verbose` is present. Contents and location are student-decided (cannot assert exact text). Tight test: no extra verbose files without the flag; some additional MC-authored files with the flag (if the loaded `<mission_control_so>` honors `verbose`).
- **Classification:** MANDATORY iff; contents OPTIONAL

### ERR-01 — Simulator must not crash except MC/Algorithm crash

- **Quote (docx):** “There is no need for the Simulator to handle a crash scenario of MissionControl / Algorithm, however in any other case the Simulator shall not crash.”
- **Setup:** Valid CLI; missing/bad args (CLI-04–07); unloadable plugin among valid ones (PLUGIN-04); mandatory wall-collision throw (FAULT-02) using a **faulty mock algorithm** that the Simulator/MC must catch.
- **Expected:** Simulator process stays alive except when a plugin actually crashes (handling of that crash is optional).
- **Classification:** MANDATORY (plugin-crash handling OPTIONAL)

### FAULT-01 — Valid algorithm must not create detectable errors

- **Quote (PDF):** “All scenarios below, using a valid algorithm” / “A valid algorithm should avoid error that it can detect.” / **Mandatory** / “The valid algorithm will not create the error.”
- **Setup:** Competition or a direct run of the submission’s `Algorithm_<ids>.so` (or a known-valid grader algorithm when testing the Simulator) on a normal composition (no injected sensor faults).
- **Expected:** No wall-collision exception path; algorithm does not itself generate the listed detectible errors. PDF gives no log line or exit code. Overlaps MAP-ALGO (“Do not fly the drone into walls”).
- **Classification:** MANDATORY (observability is weak without an independent collision oracle)

### FAULT-02 — Wall collision: Mock Movement throws; caught; process lives

- **Quote (PDF):** “A faulty algorithm returns a movement that results with the drone colliding with a wall on the actual map.” Detector: “Mock Movement driver (Which as a mock, holds the real map and can detect the collision).” Action: “Throw an exception.” **Mandatory.** Targets: “Drone Controller, SimulationRun.”
- **Quote (docx):** Simulator shall not crash except plugin crash.
- **Setup:** Comparative/competition with a **grader mock** `<algorithm_so>` that commands a move into a wall on the hidden map (maps under `inputs/map/`). Student Simulator’s Mock Movement must hold that map (Structuring PDF).
- **Expected:** Exception is thrown inside the simulator stack and caught by Drone Controller / SimulationRun. The **simulator process must not crash**. Continue vs abort of that mission after catch is **unspecified**. No prescribed log line.
- **Classification:** MANDATORY

### THREAD-01 — Missing `num_threads` or `num_threads=1` → single (main) thread

- **Quote (docx):** “If the argument is missing or if `num_threads` provided `= 1`, the program will use a single thread (the main thread).”
- **Setup:** CLI-01 without `num_threads`; CLI-01 with `num_threads=1`.
- **Expected:** Run completes correctly (same class of output files). Thread count is **not** specified on stdout — file/YAML correctness is the black-box check; `/proc` or `strace` is extra-spec.
- **Classification:** MANDATORY (stdout observability unspecified)

### THREAD-02 — `num_threads>=2` means workers **in addition to** main (total never 2)

- **Quote (docx):** “If `num_threads` provided `>= 2`, the program will interpret `<num_threads>` as the requested number of threads for running the actual simulation **in addition to the main thread**.” “Above means that the **total number of threads will never be 2**.” “the exact number of threads may be lower than requested … you should not open threads which cannot be utilized.” Main “waiting for all other threads in a join call” is OK.
- **Setup:** `num_threads=2` with enough independent jobs (several plugins and/or composition entries) to utilize workers.
- **Expected:** Correct results still produced. If process inspection is allowed: worker count + main ≠ 2. Idle extra threads must not be created when there is nothing to run — **not observable from files**.
- **Classification:** MANDATORY as a runtime rule; **not reliably testable from unzip+stdout+files alone**

### MAP-ALGO — Minimal algorithm requirements (intent)

- **Quote (docx):** “the minimal requirement for your algorithm are: Do not fly the drone into walls. Try to map all the relevant surroundings in the configured boundaries. Try to be efficient and exact.”
- **Setup:** Run submission `Algorithm_<ids>.so` on a normal composition.
- **Expected:** Must not command wall collisions (overlaps FAULT-01 / FAULT-02). “Map all surroundings” / “efficient and exact” have **no numeric threshold** in this document.
- **Classification:** MANDATORY as intent; metrics GENUINELY UNSPECIFIED

---

## Optional / bonus behaviors

### CLI-OPT-01 — `num_threads=<num>` omitted

- **Quote (docx):** syntax `[num_threads=<num>]`; “The `num_threads` argument is optional.”
- **Setup:** Omit it (THREAD-01).
- **Expected:** Legal; single main thread.
- **Classification:** OPTIONAL argument (behavior when omitted is MANDATORY)

### CLI-OPT-02 — `-verbose` omitted

- **Quote (docx):** `[-verbose]`; verbose files only “iff `-verbose` appears.”
- **Setup:** Omit `-verbose`.
- **Expected:** Legal; no MC verbose files.
- **Classification:** OPTIONAL argument

### LOG-OPT-01 — MissionControl / Algorithm **may** create error logs

- **Quote (docx):** “The MissionControl and Algorithm project **may** create error logs.”
- **Expected:** Presence is allowed, not required. Do not fail a submission for extra plugin logs, or for their absence.
- **Classification:** OPTIONAL

### BONUS-01 — Lazy load/unload of `.so`s (once)

- **Quote (docx):** “In case you find a way to avoid loading algorithms / mission_control instances simultaneously but rather load them (once!) only when needed and unload if not being used anymore (but without loading them again!) – you can ask for a bonus for that.”
- **Expected:** Not an automatic pass/fail. Asking is via whatever bonus process the course uses (older guidelines mention `bonus.txt` — **do not merge** that filename as an A3 requirement).
- **Classification:** BONUS; not auto-testable from the spec

### BONUS-02 — Class competition for “best algorithms”

- **Quote (docx):** “There would be a class competition and a bonus would be given for the best algorithms.”
- **Expected:** No formula for “best.”
- **Classification:** BONUS; metric GENUINELY UNSPECIFIED

### FAULT-OPT-01 — OOB movement ignored (not passed to Movement)

- **Quote (PDF):** “A faulty algorithm sent a movement that would result in getting out of bounds” / “Gracefully ignore the illegal movement, do not pass it to the Movement component.” **Optional.**
- **Setup:** Mock `<algorithm_so>` that requests a move leaving **bounds** (PDF distinguishes this from **mission bounds**, FAULT-OPT-08).
- **Expected if implemented:** Movement driver not invoked for that command; run continues. No required log.
- **Classification:** OPTIONAL (bonus-eligible)

### FAULT-OPT-02 — Invalid command values: retry, throw after N

- **Quote (PDF):** “A faulty algorithm returned a command with invalid values” / “Gracefully try again, throw an exception after N tries.” **Optional.** “Choosing the N value … is up to the implementation.”
- **Expected if implemented:** After some finite N, exception caught by Drone Controller / SimulationRun; simulator does not crash. Do **not** hardcode N.
- **Classification:** OPTIONAL

### FAULT-OPT-03 — NOOP (empty movement and scan): retry, throw after N

- **Quote (PDF):** “A faulty algorithm returns an empty movement and scan command (`\"NOOP\"`)” / “Gracefully handle the empty request, try again. Throw after N failed attempts.” **Optional.**
- **Setup:** Mock algorithm always returns empty `MappingStepCommand` movement and scan (`std::optional` unset) while `status` stays `Working` (header `MappingStepCommand`).
- **Classification:** OPTIONAL

### FAULT-OPT-04 — LiDAR empty vector: retry, throw after N

- **Quote (PDF):** “The LiDAR returns an empty vector” / **Optional.**
- **Setup:** Requires a Simulator that can inject empty `LidarScanResult`, or a test double — **not injectable via public CLI**. Skip unless the submission exposes a hook (it does not).
- **Classification:** OPTIONAL; typically not black-box without a custom Simulator

### FAULT-OPT-05 — Movement driver returns False: retry, throw after N

- **Quote (PDF):** “The movement driver returns `\"False\"` (for its own reasons).” Header `MovementResult.success` / `operator bool()`.
- **Same injectability limit as FAULT-OPT-04** unless testing the student MC against a grader Simulator.
- **Classification:** OPTIONAL

### FAULT-OPT-06 — Movement larger than max: split into 2+ steps

- **Quote (PDF):** “The Algorithm returned a movement bigger than the max allowed” / “Handle gracefully and separate the movements into 2 or more commands, each as a separate step.” **Optional.** Intro: “we might separate a command into 2, log the occurance as a warning and try to keep running.”
- **Setup:** Mock algorithm returns `distance` > `max_advance` / `max_elevate` from drone YAML.
- **Expected if implemented:** Command split; continue. Warning log is an example, not a required string.
- **Classification:** OPTIONAL

### FAULT-OPT-07 — Drone `"Error"` status: log and continue

- **Quote (PDF):** “The drone returned an `\"Error\"` status on step” / “Log the error, continue.” **Optional.** Detector/Targets: MissionControl. Header: `DroneStepStatus::Error`.
- **Setup:** Grader mock `IDroneControl` is **not** loadable as a plugin (MC brings its own). Observable only if student MC logs and continues when its drone step returns Error — needs a way to force that status.
- **Expected if implemented:** Some error log; mission **continues** (not abort). Log destination unspecified.
- **Classification:** OPTIONAL; weakly observable

### FAULT-OPT-08 — Mission-bounds movement amended (not ignored)

- **Quote (PDF):** “The Algorithm returned a movement that will take the drone out of the **mission bounds**” / “Amend the movement so it will not take the drone out of the mission bounds.” **Optional.** Distinct from FAULT-OPT-01 (ignore vs amend; “out of bounds” vs “mission bounds”).
- **Classification:** OPTIONAL

### FAULT-OPT-09 — GPS OOB: throw if internal also OOB, else ignore

- **Quote (PDF):** “The GPS returns out-of-bound coordinates” / “Compare with internal coordinates and throw if also OOB, else ignore.” **Optional.**
- **Injectability:** MockGPS lives in Simulator src — not a CLI feature.
- **Classification:** OPTIONAL; not CLI-black-box unless testing student MC in a grader Simulator

### FAULT-OPT-10 — GPS impossible after move: retry, then **return Error** (not throw)

- **Quote (PDF):** “A movement executed but GPS updated with impossible coordinates” / “Gracefully try again to get GPS values, **return Error** on result after N tries.” **Optional.**
- **Classification:** OPTIONAL; same injectability limit

### ERR-GRACEFUL-SEMANTICS

- **Quote (PDF):** “Gracefully handling issues is not that we don't throw an exception, is that we adjust our behavior to correct the issue or continue despite it. For example, we might separate a command into 2, log the occurance as a warning and try to keep running.”
- **Classification:** definition of “gracefully”; applies when handling optional rows

---

## Genuinely unspecified (shape-only tests)

Do **not** invent the missing rule. Test only the contract/shape.

### UNSPEC-01 — `total_score` / `total_steps` / `mission_score` formulas

- **Quote:** Docx examples `total_score: 495`, `total_steps: 100`. Header `SimulationResult.mission_score`, `SimulationManagerReport.{metric, score_range, error_score}` with defaults `error_score = -1`. No formula in any allowed source. Assignment 3 last item `[TBD]`.
- **Shape-only:** Those YAML numeric fields exist and parse as numbers; `results_summary` sort uses them in competition (YAML-OUT-02). Do not check the value against a guessed metric. In-memory `metric` / `score_range` / `error_score` are **not** keys in the comparative/competitive YAML examples — do not require them on disk.

### UNSPEC-02 — What makes two MissionControls have `same_results`

- **Quote (docx):** `same_results:` lists of manager `.so` names; sort “by number of agreeing managers, descending.”
- **Shape-only:** `same_results` is a list of strings; group sizes determine sort order. **Do not** decide equality (maps? scores? step counts? `MissionRunResult`?).

### UNSPEC-03 — Per-run YAML schema (“as in assignment 2”)

- **Quote (docx):** “YAML as in assignment 2.” Assignment 2 was excluded from this catalog. Skeleton has no output YAML sample.
- **Shape-only:** extra YAML files in the results dir; names include the plugin name. Do not assert keys such as `generated_at_utc` on those files unless you separately adopt Assignment 2 (out of scope).

### UNSPEC-04 — Retry count `N`

- **Quote (PDF):** “Choosing the N value (number of tries, see below) is up to the implementation.”
- **Shape-only:** if optional retry faults are implemented, a finite retry then throw/Error; never assert a specific N.

### UNSPEC-05 — `<time>` folder suffix and map filename pattern

- **Quote (docx):** “generate a new number per time”; map names “you should decide … easy to relate each file to the mission.”
- **Shape-only:** directory matches `comparative_results_*` / `competition_*`; map files unique and nonempty; collision avoidance across two runs.

### UNSPEC-06 — Usage / error **wording**

- **Quote (docx):** “Exact usage description and error message text in all above cases are **for your decision**.”
- **Shape-only:** usage is printed; missing/unsupported/file/folder cases mention the relevant argument **name** or path; create-dir failure writes something to the screen. No golden strings.

### UNSPEC-07 — How drones × lidars × missions are crossed

- **C++ (`SimulationCompositionData`):** `simulation_mission_groups` is `vector<tuple<SimulationConfigData, vector<MissionConfigData>>>` plus composition-level `drone_configs` and `lidar_configs`.
- **Sample YAML:** drones/lidars listed once under `simulation_compositions`, missions listed per `simulation_config`.
- **No sentence states** “run every drone with every lidar with every mission.”
- **Shape-only:** a composition with 2 drones, 2 lidars, and 2 missions produces **some** number of runs ≥ 1; do not fail a team for 2 vs 4 vs 8 runs unless a later spec defines the product. Docx “all the configurations in the simulation yaml config” does not define the unit.

### UNSPEC-08 — `dimensions_cm` vs `DroneConfigData.radius`

- **YAML:** `dimensions_cm` with comment “sphere diameter drone can pass through.”
- **Header:** `PhysicalLength radius`.
- **Shape-only:** YAML key `dimensions_cm` is accepted. Do not assert radius = cm/2 vs radius = cm.

### UNSPEC-09 — `output_mapping_resolution_factor`

- **Header `MissionConfigData`:** `double output_mapping_resolution_factor = 0.0`.
- **No mission YAML sample contains this key.** YAML spelling unspecified. `ResolutionRequestStatus { Accepted, Ignored, IgnoredTooSmall }` has no YAML mapping.
- **Shape-only:** missions without that key still run (samples omit it).

### UNSPEC-10 — Error log names, log line format, `ErrorRef.code` vocabulary

- **Quote (docx):** “Error log(s) files.” Header `ErrorRef { code, message }` with no enum.
- **Shape-only:** ≥1 error-log file in the results folder. Do not require specific filenames or `ErrorRef.code` strings.

### UNSPEC-11 — Map file format

- **Inputs** are `.npy` (and `.cw` exist under `inputs/map/`). `IMutableMap3D::save(path)` has no format in the header. Docx does not specify output map format.
- **Shape-only:** unique files exist and are attributable to missions. Do not require `.npy` vs `.cw` vs YAML.

### UNSPEC-12 — Summary YAML **filename**

- Docx names the artifact “Comparative/Competitive Simulation Result Output File (YAML)” but never a basename. `.gitignore` has `simulation_output.yaml` (hint only).
- **Shape-only:** exactly one file in the results dir parses as `comparative_report` / `competitive_report`. Do not require a specific name.

### UNSPEC-13 — Invalid YAML **content** (parseable file, bad schema)

- No Assignment 3 sentence. CLI-06 covers unreadable/missing files only.
- **No test** for “wrong keys” without inventing a policy.

### UNSPEC-14 — Exit codes; stdout vs stderr

- Docx: “print”, “write … to screen”, “finish.” No `EXIT_FAILURE`.
- **Shape-only:** process ends after usage/error cases; do not require a specific code or stream.

### UNSPEC-15 — `num_threads` of 0, negative, or non-integer; both mode flags; neither mode flag

- Not specified.
- **No test** that asserts a particular handling (beyond CLI-05 if an extra/unknown token is treated as unsupported).

### UNSPEC-16 — Timeout / wall clock / max runtime

- Assignment 3: **none**. Error Code Key `b05` “timeout on scenario (1 minute)” is a **review spreadsheet** item, not A3 prose — do not merge as an A3 CLI requirement. Header `max_steps` **is** a mission YAML field (samples include it) — that is a **step cap in the mission**, not a process timeout.

### UNSPEC-17 — Thread count on stdout

- No log format for threads. THREAD-01/02 are behavioral, not line-oriented.

---

## Static/structural checks on the unzipped submission (not runtime-testable)

These are graded by inspecting the tree / build products, not by “run and read YAML” alone. A tester that only executes `simulator_<ids>` will miss them.

A generic tester needs to **discover and invoke whatever build tooling the submission provides**, since it may differ per team. The skeleton has CMake + `CMakePresets.json` + vcpkg, but a submission may replace that with Makefiles or another system. The only portable post-build contract is the artifact names.

### ZIP-01 — Archive name

- **Quote (docx):** `ex3_<student1_id>_<student2_id>.zip` → `ex3_<ids>.zip`.
- **Kind:** static (the zip itself)

### ZIP-02 — Five folders

- **Quote (docx):** `Simulator`, `Algorithm`, `MissionControl`, `common`, `UserCommon`.
- **Note:** this **skeleton has no `UserCommon/`**. Zip rule comes from the docx (skeleton does not encode zip contents). Students must add `UserCommon/`.

### ZIP-03 — Four build files; none student-added in `common/` / `UserCommon/`

- **Quote (docx):** “4 makefiles (or CMakes) - 3 for each of the projects, inside the folder of the project that it builds. Additional one at the root … for building all 3 projects.” “there are no makefiles in these folders!” (`common`, `UserCommon`).
- **Skeleton conflict:** staff **already** ships `common/CMakeLists.txt` (`drone_common` INTERFACE). That staff file is “as-is,” not a student makefile in `common/`. Root `CMakeLists.txt` `add_subdirectory(common)` is also staff. See contradictions.

### ZIP-04 — `students.txt`

- **Quote (docx):** “one line per submitter with name and id.” Skeleton placeholders: `TODO: Student name, student ID` (two lines). Exact grammar unspecified.

### ZIP-05 — `README.md` at zip root

- **Quote (docx):** “directly in the zip without any folder”; “info and remarks about your implementation.” Skeleton: “You should update this README file.”

### ZIP-06 — No binaries / no unapproved external libraries in the zip

- **Quote (docx):** “DO NOT SUBMIT … binary files” / “external libraries (you may only use standard C++ libraries or libraries which were explicitly approved in the course forum).”
- Skeleton `.gitignore` ignores `*.so`, `*.exe`, `build/`. `vcpkg.json` lists `mp-units`, `yaml-cpp`, `tinynpy`, `gtest` as **project deps** (not zip contents). What counts as “binary” at the margin (e.g. `inputs/map/*.npy`) is **ambiguous**; maps are required inputs in the skeleton.

### ZIP-07 — `common/` as-is, no extra student files

- **Quote (docx):** published files “as is” without any change. “DO NOT add your own files into this folder!”
- **Check:** header-for-header diff vs this skeleton’s `common/include/` (and `common/CMakeLists.txt`). Frozen interfaces include registration macros.

### ZIP-08 — Frozen Simulator / MissionControl published headers unchanged

- Paths: `Simulator/common_simulator/include/Simulator/{ISimulation,ISimulationRun,ISimulationRunFactory,SimulationTypes}.h`, `MissionControl/common_mission_control/include/MissionControl/IDroneControl.h`.
- Structuring PDF: MC common “includes only the `IDroneControl`”; MC “bring its own implementation”; MissionControl and MappingAlgorithm **interfaces** live in shared `Common`.

### ZIP-09 — Artifact names after **their** build

- **Quote (docx):** executable `simulator_<ids>`; `Algorithm_<ids>.so`; `MissionControl_<ids>.so`.
- Skeleton CMake has **no** `OUTPUT_NAME` / no `add_executable`. Algorithm comment: SHARED + “remove the usual `lib` output prefix.” MissionControl comment: SHARED, **no** PREFIX note. Simulator comment: executable + `${CMAKE_DL_LIBS}` + export registration constructors.
- **Check:** after invoking the submission’s build, those three names exist. **Generic tester must discover the build system** (CMake, Make, or other). Keeping `CMakePresets.json` (`default`, Ninja, `build/default`, `VCPKG_ROOT` toolchain) is **not** required by the docx.

### ZIP-10 — Three separately built parts; plugins are SHARED `.so`

- **Quote (docx):** “separated into three parts, each … built separately”; parts 1 and 2 “compiled as shared libraries (`.so`) that would be dynamically loaded by the Simulation project.”

### ZIP-11 — Registration macros in plugin sources

- **Quote (headers, as-is):** `REGISTER_MAPPING_ALGORITHM(class_name)` → `::common::MappingAlgorithmRegistration register_me_##class_name{...}`; `REGISTER_MISSION_CONTROL(class_name)` similarly. Docx: headers as-is; registration `.cpp` **in Simulator only**.
- **Check:** student Algorithm/MC `.cpp` uses those macros; Simulator provides the constructor definitions.

### ZIP-12 — Namespaces (inspect source)

- **Docx:** `algorithm_<ids>`, `mission_control_<ids>`, `user_common_<ids>`.
- **README:** “lowercase project namespaces `common`, `algorithm`, `mission_control`, and `simulator`.”
- **Headers (win for published APIs):** `common`, `common::types`, `mission_control` (`IDroneControl` only), `simulator`, `simulator::types`. **No `namespace algorithm` in any frozen header** (`IMappingAlgorithm` is `common::IMappingAlgorithm`).
- Student implementation namespaces may follow the docx; they must still implement the frozen `common::` / `mission_control::` / `simulator::` types.

### ZIP-13 — Mocks live in Simulator `src`, not in Algorithm

- **Quote (Structuring PDF):** Simulator `src` includes Mock Lidar, MockGPS, MockMovement, Map3DImpl. “MockLiDAR needs to hold the real, hidden map.”
- **Check:** those impls are under Simulator, not shipped inside `Algorithm_<ids>.so`.

### ZIP-14 — `UserCommon` namespace / shared student code

- **Quote (docx):** student shared code; namespace `user_common_<ids>`. Not in the skeleton.

### ZIP-15 — No `new` / `delete`

- **Quote (docx):** “As in assignment 2, you are not allowed to use `new` and `delete` in your code.”
- **Check:** source grep (not a binary run). AdvCpp also: avoid `malloc`/`free`; rule of 3.

### ZIP-16 — C++20 / warning flags (skeleton + older guidelines)

- Skeleton root: `CMAKE_CXX_STANDARD 20`; unused helper `drone_warnings` = `-Wall -Wextra -Werror -pedantic`. Older Submission Guidelines (do not merge as A3): `g++ -std=c++20 -Wall -Wextra -Werror -pedantic`. Treat as **build-time** if the team kept the skeleton function and actually calls it — skeleton never calls `drone_warnings()`.

### ZIP-17 — Staff `inputs/` tree

- Present in the skeleton, **omitted** from README’s “Provided file tree.” Not listed in the zip-required contents. Whether submissions must include sample maps/YAMLs is **unspecified** by the docx.

### Frozen interface signatures (for grader mock plugins)

These are how a grader **constructs** `<algorithm_so>` / `<mission_control_so>`; they are not YAML/CLI assertions. Skeleton wins.

- `common::IMappingAlgorithm::nextStep(const DroneState&, const LidarScanResult*)` → `MappingStepCommand` (`optional` movement + scan, `AlgorithmStatus`). Comment: “Algorithms may inspect the map but must not edit it directly.” (`const IMap3D&`).
- `common::IMissionControl::runMission()` → `MissionRunResult`.
- `mission_control::IDroneControl::{step, state}`.
- `simulator::ISimulation::run(composition, output_path)` → `SimulationManagerReport`.
- `simulator::ISimulationRun::run()` → `SimulationResult`.
- `simulator::ISimulationRunFactory::create(sim, mission, drone, lidar, output_path)`.
- `IMap3D` / `IMutableMap3D::{set,save}`; `IGPS`; `ILidar`; `IDroneMovement::{rotate,advance,elevate}`.
- Comment: “mission will create its own drone controller.”

Enums / result types: `RotationDirection`, `MovementCommandType`, `AlgorithmStatus`, `DroneStepStatus { Continue, Completed, Error }`, `VoxelOccupancy`, `MissionRunStatus { Completed, MaxSteps, Error }`, `ResolutionRequestStatus { Accepted, Ignored, IgnoredTooSmall }`, `MovementResult`, `DroneStepResult`, `ErrorRef { code, message }`, `MissionRunResult`.

---

## Not testable by any automated suite (manual review items)

Do not encode these as runtime tests. Source: AdvCpp Review Guideline, Error Code Key `e*`, Known Issues process, plus code-quality sentences in Assignment 3 / PDFs.

### Manual — design / style (AdvCpp “Code review”)

Quoted: coherent project structure; avoid unnecessary `#include`s (reasonable DAG); const-correctness, encapsulation, lean classes; extract non-member logic to anonymous namespaces; “avoid using `new` and `delete` where possible, and avoid C-like `malloc` and `free` entirely”; rule of 3; no magic numbers; avoid expensive recomputations; correct ref/const-ref; std containers / smart pointers; short single-responsibility functions; “HLD and design docs should match the design of the code.” Frontal design review “published at a later date.”

### Manual — Error Code Key `e*` (weights minor/normal/severe)

| index | Error Description |
|---|---|
| e01 | Unrelated grouped classes in header |
| e02 | Too granular header separation |
| e03 | Unwrapping mp-units types for mathematical operations |
| e04 | Incorrect use of c++ stdlib |
| e05 | passing diagnostic message into handler function |
| e06 | passing parameters that can be const-ref as mutable-ref/copy |
| e07 | class methods could be const |
| e08 | unneeded methods / data members in class API |
| e09 | Too long functions |
| e10 | Duplicate class, functions or logical flows |
| e11 | library integration requires / uses manual install |
| e13 | incorrect use of pointers |
| e14 | Incomplete class diagram in HLD |
| e15 | incomplete sequence diagram in HLD |
| e16 | using numerical fields / parameters instead of strong types (e.g. f(double x_cm)) |
| e17 | Unnecessary dependancy |
| e18 | Noncompliant submission |
| e21 | repeated computation |
| e22 | Poor encapsulation |
| e23 | Use of magic numbers |

No `e12`/`e19`/`e20` in the sheet.

### Manual — Assignment 3 code rules with no I/O

- “Do not cache instances, just recreate them using the factories when needed” (not the same as avoiding reload of the same `.so`).
- `dlclose` before exit; do not `dlclose` while objects from that `.so` are alive.
- “It is better not to lock if you can avoid locking. But if you need to lock, you should of course lock.”
- Registration constructors implemented “as you wish” in Simulator `.cpp`.
- Algorithm goals without a numeric bar: “Try to map all the relevant surroundings… Try to be efficient and exact.”

### Manual — HLD PDF

- Older Submission Guidelines require an HLD PDF in the submission root — **older exercise; do not merge as A3**. Error Code Key still scores HLD diagrams (`e14`/`e15`). Assignment 3 zip list does **not** name an HLD file.

### Manual — Known Issues.xlsx (optional process)

- Assignment 3-titled process doc; examples are “from another project.” Optional excel; English; may affect how a **human** checker treats listed bugs. Not a runtime spec. “Is this file mandatory? No, but it may improve your grade.”

### Flag-only — Submission Guidelines (older `drone_mapper` exercise)

Do **not** test: `drone_mapper [<input_output_files_path>]`, `map_output.txt`, three input-set folders with nested `original_output`, zip name `ID1_ID2.zip` (A3 is `ex3_<ids>.zip`). May still inform humans: gcc 11.4+, Makefile or CMake, no binaries, std libraries unless forum approval, readme with names/IDs and I/O format, optional `bonus.txt`.

### Flag-only — Error Code Key `b*` (build/run, but not A3 spec)

These *can* be exercised by a harness, but they are spreadsheet labels, not Assignment 3 sentences: `b01` Build failure of main; `b02` Build failure of extra; `b03` missing scenario validation; `b04` error on scenario; `b05` timeout on scenario **(1 minute)**; `b06` no error inputs on missing input; `b07` manual config change failure; `b08` scenario required manual change. **Do not treat “1 minute” as an A3 requirement.**

---

## Contradictions / ambiguities found across sources

**Winner rule:** skeleton headers/CMake/README win on anything they actually define; otherwise Assignment 3 docx; PDF for fault mandatory/optional.

### C1 — Plugin/executable names: docx vs skeleton CMake

- **Docx:** `simulator_<ids>`, `Algorithm_<ids>.so`, `MissionControl_<ids>.so`.
- **Skeleton CMake:** no `OUTPUT_NAME`, no executable target, only comments (Algorithm: SHARED + drop `lib` prefix; MC: SHARED; Simulator: executable + `dl` + export registrations).
- **Winner for testers:** use the **docx names** after build (CMake does not encode a conflicting name; it encodes nothing). Flag: PREFIX stripping is only commented for Algorithm, not MissionControl.

### C2 — Namespaces: README vs headers vs docx

- **README:** `common`, `algorithm`, `mission_control`, `simulator`.
- **Headers:** `common` / `common::types`, `mission_control` (`IDroneControl`), `simulator` / `simulator::types`. No `algorithm` namespace; mapping API is `common::IMappingAlgorithm`.
- **Docx:** `algorithm_<ids>`, `mission_control_<ids>`, `user_common_<ids>`.
- **Winner:** frozen **header namespaces** for anything a mock plugin must compile against. Student code namespaces follow the docx/README as implementation detail. README `algorithm` is **not** a published header namespace.

### C3 — “All CLI arguments are mandatory” vs optional `num_threads` / `-verbose`

- **Docx:** “All command line arguments are mandatory.” **and** “The `num_threads` argument is optional.” **and** `[num_threads=<num>] [-verbose]`.
- **Winner:** treat bracketed args as optional (later sentences + syntax). Mark the blanket “all mandatory” line **ambiguous**. Tests: omitting `simulation=` fails; omitting `num_threads` succeeds.

### C4 — Comparative vs competition results **folder names**

- Comparative: `comparative_results_<time>`. Competition: `competition_<time>` (no `_results_`). Not a conflict — easy to get wrong in tests. **Use the strings verbatim.**

### C5 — Where outputs go

- **Docx:** directly under `mission_control_folder` / `algorithms_folder`. No `--output`.
- **Header `ISimulation::run(..., output_path)`** is an in-process API, not a CLI flag.
- **`.gitignore`:** `simulation_output.yaml` — not a CLI.
- **Winner:** on-disk location = docx (under the plugin folders). Do not require cwd output.

### C6 — In-memory `SimulationManagerReport` vs on-disk comparative/competitive YAML

- **Header** `SimulationManagerReport`: `composition_file`, `generated_at_utc`, `metric`, `score_range`, `error_score`, `runs` (`vector<SimulationResult>`).
- **Docx YAML:** `comparative_report` / `competitive_report` with `results_summary`, `same_results` / `algorithm`, `total_score`, `total_steps`, `errors`, plus `mission_control_folder` or `mission_control`.
- **Winner:** **CLI file YAML = docx**. C++ struct = Simulator internal API (not a substitute schema). Do not require `metric` / `score_range` / `error_score` / `runs` on disk.

### C7 — `UserCommon/` required in zip, absent from skeleton/README tree

- **Docx** requires it. **Skeleton** does not contain it.
- **Winner:** zip must contain `UserCommon/` (docx governs submission contents). Skeleton is incomplete relative to the zip list — flag, don’t fail the skeleton.

### C8 — Makefiles in `common/`

- **Docx:** no makefiles in `common/` or `UserCommon/`.
- **Skeleton:** `common/CMakeLists.txt` exists (staff INTERFACE lib).
- **Winner:** keep staff `common/CMakeLists.txt` as-is (skeleton + “as-is”). Students must not **add** more build files there. Root CMake that `add_subdirectory(common)` is staff.

### C9 — `IDroneControl` namespace vs README “MissionControl”

- **Header:** `namespace mission_control { class IDroneControl }` with `using namespace common;` inside the header.
- **CMake:** expose `common_mission_control/include` **privately** to MissionControl — not on `drone_common` INTERFACE.
- No conflict with Structuring PDF (“common includes only the IDroneControl”).

### C10 — YAML keys vs C++ field names (input)

- Drone: YAML `dimensions_cm` vs C++ `radius`.
- Simulation: YAML `map_resolution_cm`, `map_axes_offset.{x_offset,y_offset,height_offset}`, `height_cm`, `initial_angle_deg` vs C++ `map_resolution`, `map_offset` (`x,y,z`), `initial_angle`.
- Mission: YAML nested `*_boundary.min_cm` vs C++ `MappingBounds.{min_x,max_x,...}`; YAML omits `output_mapping_resolution_factor`.
- **Winner for files:** YAML keys from skeleton samples. **Winner for plugins:** C++ names. Mapping between them is unspecified (UNSPEC-08/09).

### C11 — README file tree omits `inputs/`

- Skeleton on disk has `inputs/`. README “Provided file tree” does not. Layout check should not require README and disk to match for `inputs/`.

### C12 — Competition “all composition configs”

- Stated only for comparative. Competition sentence is only “run the given MissionControl using all algorithms.”
- **Ambiguous.** Shape-only: at least all algorithms run; do not fail if they also iterate the full composition (likely intended).

### C13 — Folder “zero files of the desired usage”

- No glob. If the loader only considers `*.so`, a folder of `.txt` is “zero files.” If it tries every file, CLI-07 vs PLUGIN-04 overlap.
- **Ambiguous.** Test with a truly empty directory (must usage-fail) and with a mix of good+bad `.so` (must run + `errors:`).

### C14 — Wall collision: throw (PDF mandatory) vs “shall not crash” (docx)

- Not a true conflict if the exception is **caught** (PDF Targets = Drone Controller, SimulationRun). **Winner:** throw + catch; process lives.

### C15 — “Out of bounds” vs “mission bounds” (PDF rows 2 vs 10)

- Different optional actions (ignore vs amend). Tests must not conflate them. “Bounds” without qualifier is **ambiguous**.

### C16 — Plugin crash vs “could not be … run” in `errors:`

- Docx: no need to handle MC/Algorithm **crash**; YAML `errors` comment is “could not be loaded / run.”
- **Ambiguous** whether a crashing plugin must appear in `errors` (handling not required).

### C17 — Draft / `[TBD]` / YAML `[...]`

- Extra summary-YAML keys after `[...]`: not required, not forbidden.
- Last Additional Notes item `[TBD]`: **no requirement**.

### C18 — Older Submission Guidelines vs A3 zip/CLI

- `drone_mapper`, `map_output.txt`, `ID1_ID2.zip` vs `simulator_<ids>`, `ex3_<ids>.zip`, comparative/competition CLI.
- **Winner:** Assignment 3. Keep older doc as flag-only.

### C19 — Error Code Key 1-minute timeout vs A3 (no timeout)

- **Winner:** A3 has no process timeout. Do not fail runs at 60s unless course staff separately adopt `b05`.

### C20 — `ISimulation` output_path vs no CLI `--output`

- `ISimulation::run(composition, output_path)` exists. CLI has no matching flag; results dir is derived from `mission_control_folder` / `algorithms_folder`.
- **Winner:** CLI = docx. Header is the in-process API the Simulator implementation must satisfy internally.

### C21 — Algorithm `lib` prefix vs MissionControl

- Only Algorithm CMake comment says remove `lib`. Linux default would emit `libMissionControl_<ids>.so` if PREFIX is left on.
- **Winner for testers:** docx `MissionControl_<ids>.so` / `Algorithm_<ids>.so` (no `lib`). Flag the CMake comment gap.

### C22 — How many `.so`s per component

- CMake: “each … implementation as a SHARED library.” Docx examples are one Algorithm `.so` and one MissionControl `.so` named with `<ids>`. Folders in comparative/competition hold **many teams’** plugins.
- **Ambiguous** whether one submission may ship multiple Algorithm `.so`s. Tester should still accept a folder of third-party plugins at CLI time.

---

### “Your decision” items (loose checks only)

| Topic | Docx | What a test may check |
|---|---|---|
| Usage text | “for your decision” | Printed; mentions missing/unsupported arg names |
| All error strings | “for your decision” | Nonempty message; then finish / screen |
| `<time>` | “new number”; coliru link not inlined | Unique dir; pattern prefix `comparative_results_` / `competition_` |
| Map filename pattern | “easy to relate … to the mission” | Unique; some token tying file to a mission is **subjective** — do not golden-name |
| Verbose file contents | “anything you think is of interest” | Presence iff `-verbose` |
| Error-log filenames | “Error log(s) files” | ≥1 file in results dir |
| Registration `.cpp` | “as you wish” | Plugins actually register (runtime success) |

---

## Appendix — Mandatory vs optional fault table (quoted from Common issues PDF)

Intro: “You are required to handle the Mandatory ones only. The optional ones might achieve a bonus.” “Choosing the N value (number of tries, see below) is up to the implementation.”

| # | Scenario | Action | Mand./Opt. | Targets |
|---|---|---|---|---|
| 1 | All scenarios below, using a valid algorithm | A valid algorithm should avoid error that it can detect. | Mandatory | The valid algorithm will not create the error. |
| 2 | Faulty algorithm movement would get out of bounds | Gracefully ignore; do not pass to Movement. | Optional | Drone controller |
| 3 | Faulty algorithm command with invalid values | Retry; throw after N tries. | Optional | Drone Controller, SimulationRun |
| 4 | Empty movement and scan (“NOOP”) | Handle empty request, retry; throw after N. | Optional | Drone controller, SimulationRun |
| 5 | Movement collides with a wall on the actual map | Throw an exception. | Mandatory | Drone Controller, SimulationRun |
| 6 | LiDAR returns an empty vector | Retry; throw after N. | Optional | Drone Controller, SimulationRun |
| 7 | Movement driver returns “False” | Handle; throw after N. | Optional | Drone Controller, SimulationRun |
| 8 | Movement bigger than max allowed | Split into 2+ commands, each a separate step. | Optional | Drone Controller |
| 9 | Drone returned “Error” status on step | Log the error, continue. | Optional | MissionControl |
| 10 | Movement would leave mission bounds | Amend so it will not leave mission bounds. | Optional | Drone Controller |
| 11 | GPS returns OOB coordinates | Throw if internal also OOB, else ignore. | Optional | Drone Controller, SimulationRun |
| 12 | GPS updated with impossible coordinates after move | Retry GPS; return Error after N tries. | Optional | Drone Controller |
