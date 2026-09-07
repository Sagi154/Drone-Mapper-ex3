---
name: zip-submission
description: Use when packaging or creating the ex3 submission zip (ex3_207190406_209543255.zip), staging a Linux zip tree, or checking Windows/Linux zip mismatch — CRLF, backslash paths, extra root folder, Compress-Archive, git archive, or binaries in the archive.
---

# Zip Submission

Produces `ex3_207190406_209543255.zip` from a **Linux staging tree**. This skill **packages**. `pre-submission-review` **inspects**. `verify-submission-readiness` is the test dashboard — not a zip.

Announce: “Using zip-submission.”

Authoritative include/omit list: `docs/submission-junk-audit.md`. Do not invent extras. Do not copy the audit into the zip.

## Hard rules

- Do **not** zip the git working copy.
- Do **not** use `git archive` (ships `.cursor/`, `AGENTS.md`, `docs/`, `context/`, `scripts/`).
- Do **not** use PowerShell `Compress-Archive`.
- Do **not** rewrite line endings in git / the working tree. Convert **only** in the stage.
- Zip **from inside** the stage so entries sit at the archive root. No wrapper folder `ex3_207190406_209543255/`.
- Name must be exact: `ex3_207190406_209543255.zip`.
- Execute `scripts/stage_and_zip.sh`. Do not regenerate a parallel zipper.

**PowerShell `Compress-Archive` is the usual failure mode:** backslash paths, extra nesting, and a format Linux unzip sometimes hates.

## How we should zip

Build a Linux staging tree inside Docker, then zip from inside that tree so entries sit at the archive root.

Stage (Docker, bind-mount the repo) into something like `/tmp/ex3_stage/` — only the junk-audit include list:

- five folders, minus `tests/manual/`, ASSUMPTIONS.md, `tiny_*.yaml`, skeleton_host’s standalone `CMakeLists.txt` + ASSUMPTIONS.md, all `.gitkeep`
- root: `CMakeLists.txt`, `CMakePresets.json`, `vcpkg*.json`, `README.md`, `students.txt`, `HLD.pdf`, `bonus.txt`, `Known Issues.xlsx`
- `inputs/` minus `profile_cell.yaml` (drop `.cw` / `npy_to_cw.py` if you want a stricter zip)

Normalize text to LF in the stage (not in git). Skip binaries: `.npy`, `.pdf`, `.xlsx`.

Zip on Linux:

```bash
cd /tmp/ex3_stage
zip -r /work/ex3_207190406_209543255.zip .
```

Copy the archive to the repo (or Desktop). Name must be exact. No wrapper folder `ex3_207190406_209543255/`.

`.gitattributes` only forces LF on `*.sh`. Everything else can be CRLF on this host, which is why conversion happens in the stage, not by rewriting the working tree.

Default for this repo: **keep** `inputs/map/*.cw` and `npy_to_cw.py` (they are in the ex3 skeleton). Drop only `inputs/profile_cell.yaml`. Stricter zip: set `STRICT_INPUTS=1` on the script.

Do **not** drop C++ test / fixture / `skeleton_host` **sources**. The default `cmake --preset default` build still compiles them. Omit-list only: see the junk-audit.

## Windows/Linux mismatch — what actually bites graders

| Risk | Harm | How we check |
|------|------|--------------|
| Zip created on Windows | `\` in paths, extra root folder, unzip fails | `unzip -l` inside Docker; first entries must be `Algorithm/`, `Simulator/`, `README.md`, … never `ex3_…/Simulator/` |
| CRLF in sources | Usually gcc-ok; can break CMake/g++ `-x`, YAML, or a grader’s `file`/`dos2unix` scripts | After extract: `grep -rIl $'\r'` on the stage — must be empty for text |
| UTF-16 / BOM (`students.txt`, CMake) | Linux tools read garbage | `file students.txt README.md CMakeLists.txt` → UTF-8, no BOM |
| Accidental `.exe` / `.so` / `.o` | ZIP-06 fail | `find` on the extracted tree |
| Junk dirs (`.cursor`, `docs`, `build`) | Process notes + binaries | `unzip -l` must not list them |
| LF-only `.sh` | We omit `tests/manual/*.sh`, so this is mostly moot | still convert remaining text |

## Preconditions

Stop if any are missing — do not zip a partial tree:

- [ ] `Known Issues.xlsx` at repo root (`populate-known-issues` export). Never ship `docs/known-issues.md`.
- [ ] `students.txt` has real IDs, no `TODO:`.
- [ ] `HLD.pdf`, `bonus.txt`, `README.md` at repo root.
- [ ] Docker image `drone-mapper-ex3-dev` is available.

## Procedure

Copy this checklist:

```
Task Progress:
- [ ] Preconditions
- [ ] Stage + zip inside Docker via scripts/stage_and_zip.sh
- [ ] Listing / extract checks PASS
- [ ] Gold cmake from the extracted zip PASS (unless user set SKIP_BUILD=1)
- [ ] pre-submission-review §5d on the produced zip
```

### Run the script

Execute [scripts/stage_and_zip.sh](scripts/stage_and_zip.sh) **inside** `drone-mapper-ex3-dev`. Do not paste a new zipper into `tmp/`.

From this Windows host (PowerShell — set `MSYS_NO_PATHCONV` so Git-bash does not mangle `/work`):

```powershell
$env:MSYS_NO_PATHCONV = "1"
docker run --rm -e VCPKG_ROOT=/usr/local/vcpkg `
  -v "<repo>:/work" -w //work drone-mapper-ex3-dev bash -lc `
  "sed -i 's/\r$//' /work/.cursor/skills/zip-submission/scripts/stage_and_zip.sh && bash /work/.cursor/skills/zip-submission/scripts/stage_and_zip.sh"
```

`<repo>` is the `Drone-Mapper-ex3` root. The script writes `/work/ex3_207190406_209543255.zip`.

Optional env (prefix on the `bash` invocation):

| Env | Effect |
|-----|--------|
| `SKIP_BUILD=1` | Skip gold `cmake --preset default` (listing/CRLF/encoding/binary checks still run) |
| `STRICT_INPUTS=1` | Also drop `inputs/map/*.cw` and `npy_to_cw.py` |

If `zip` is missing in the image, the script uses Python `zipfile` with **forward-slash** names. Same idea: `cd` into the stage, then archive `.`.

## Verify after the zip exists (still in Docker)

`unzip -l ex3_207190406_209543255.zip` — ZIP-01/04/05, no junk, forward slashes.

Extract to a clean dir; `find` for `*.so`/`*.o`/`*.exe`; `grep` for CR; `file` on root text files.

Gold check: `cmake --preset default && cmake --build --preset default` from the **extracted zip**, not from this repo. That is the real Windows/Linux proof.

The script already runs these. On PASS it prints `ZIP_READY`. Then invoke `pre-submission-review` §5d against the produced archive (ZIP-01/04/05/06). Do not treat a working-tree review as a zip review.

Copy to Desktop only after PASS, keeping the exact filename.

## Red flags — stop

| Excuse | Reality |
|--------|---------|
| “Compress-Archive is fine on this machine” | Graders unzip on Linux. Backslashes / extra folder / format mismatch fail ZIP-04. |
| “I’ll zip the repo and exclude build/” | Not enough. `.cursor/`, `docs/`, `.cache/`, `.vscode/` still ship. |
| “git archive is cleaner” | Still ships agent dirs unless every path is filtered. Use the stage. |
| “I’ll dos2unix the working tree” | Pollutes git. Convert only in `/tmp/ex3_stage`. |
| “Skip Docker; we’re on Windows” | That is the mismatch. Stage and zip in Linux. |
| “Skip the extracted cmake; listing looks good” | Listing cannot prove CMake/g++ will eat the archive. Gold check is the proof. |

## After packaging

Do not commit the zip unless the user asks. `Known Issues.xlsx` at repo root is a zip-time export — do not treat it as source of truth (`docs/known-issues.md` is).
