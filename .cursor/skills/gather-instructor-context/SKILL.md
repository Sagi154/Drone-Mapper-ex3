---
name: gather-instructor-context
description: Extracts TAU Assignment 3 instructor requirements from context/ docx/pdf/xlsx via parallel subagents, then optionally audits the tree against those catalogs. Use when the user points at context/ files, asks to learn or re-diff the assignment from the course documents, wants a compliance check against instructor instructions, or before submission re-read of the Assignment 3 docx.
---

# Gather Instructor Context

`context/` is the source of truth. Condensed files under `docs/` (`assignment3-checklist.md`,
`error-handling-matrix.md`, `review-error-codes.md`, `known-issues-guidelines.md`) are **not**.
Extract from the originals first, then note where the condensations omit or add.

Do not start by reading the implementation. Do not invent requirements the originals do not state.

## When to invoke

- User `@`-mentions files under `context/` or asks to learn the assignment from the instructor docs.
- Compliance / “do we follow the instructions / no deviation” audits.
- Re-diff before submission (Assignment 3 docx left draft with a `[TBD]` bullet).
- Forum or staff may have changed a published doc.

**Not this skill:** adding Known Issues rows (use `populate-known-issues`); frozen-header git diff
(use `verify-frozen-interfaces`); zip layout (use `pre-submission-review`). Those run **after**
extraction if still needed.

## 1. List `context/` — do not trust glob alone

`context/` can be gitignored. Glob may return nothing. Always list the directory:

```powershell
Get-ChildItem -Force context
```

Expect at least:

| File | Role |
|------|------|
| `Advanced Topics TAU 2026B - Assignment 3.docx` | **Wins** on zip name, CLI, folders, plugins, threading, outputs |
| `Common issues and handling.pdf` | Mandatory vs optional fault table |
| `Structuring the project.pdf` | Why mocks live in Simulator; `IDroneControl` in MC-common |
| `Submission Guidelines - Advanced Topics Assignments - 2026B.docx` | gcc flags, HLD PDF, no binaries, stdlib/forum libs — **also contains Ex1 leftovers** |
| `AdvCpp Review Guideline.docx` | Qualitative happy-flow / code-review (no `e*`/`b*` codes) |
| `Error Code Key.xlsx` | Actual `e01`–`e23` / `b01`–`b08` point table |
| `Known Issues.docx` | Optional excel schema — extract only if the user asked; filling rows is `populate-known-issues` |

Also present and **not** Ex3 procedure: Assignment 1/2 docx, `Exercise 2 Review Guideline.docx`
(bug-injection). Ignore unless the user names them.

## 2. Extract text (parent may do this in parallel with subagents)

`.docx` is a zip. PDFs need a library. Scratch extracts may go under `tmp/` (gitignored); do not
commit them.

**docx:**

```python
import zipfile, re, html, os
path = r"context/Advanced Topics TAU 2026B - Assignment 3.docx"
xml = zipfile.ZipFile(path).read("word/document.xml").decode("utf-8")
xml = re.sub(r"</w:p>", "\n", xml)
xml = re.sub(r"</w:tr>", "\n", xml)
xml = re.sub(r"</w:tc>", " | ", xml)
text = html.unescape(re.sub(r"<[^>]+>", "", xml))
```

**pdf:** `from pypdf import PdfReader` (or `fitz`). Tables often scramble; keep detector / action /
mandatory-vs-optional from cell order.

**xlsx (error codes):** open with Python `openpyxl` / `pandas` if present; otherwise the condensation
`docs/review-error-codes.md` may be used **only after** confirming it matches the sheet.

Word tables (`w:tbl`) may be absent — YAML/CLI in Assignment 3 are ordinary paragraphs.

## 3. Dispatch one extract subagent per document — same turn

Issue **all** extract `Task` calls in **one** message (`subagent_type: generalPurpose`). One agent
per file (or Common-issues + Structuring together — they are short PDFs). Isolated context: give
each agent the **absolute path**, extract recipe, and output contract. They never inherit the
parent’s chat.

Each extractor must:

1. Pull **complete** text (tables, CLI, YAML keys, macros, `[TBD]`, draft dates, highlights).
2. Return a catalog: `ID`, near-verbatim requirement, category, mandatory vs optional/bonus, how a
   reviewer would check it.
3. Flag contradictions, underspecification, and leftover terminology.
4. **Not** look at our implementation. **Not** treat `docs/*.md` as truth — they may note checklist
   omissions **after** extracting.

**Prompt skeleton** (fill path + focus):

```text
Extract FULL requirements from <absolute path>. This is a .docx/.pdf.
Use unzip of word/document.xml or pypdf. Project: <ex3 root>.
Return an exhaustive catalog (IDs A1… / CI1… / SG1… / SP1…). Quote CLI, YAML keys,
macros, folder names verbatim. Flag [TBD], contradictions, draft notes.
Do NOT inspect the implementation. Do NOT trust docs/ condensations as source.
```

Keep working while they run (list tree, `students.txt`, CMake names). Do not poll transcript files.

## 4. Authority when documents disagree

Order from `.cursor/rules/project-context.mdc`:

1. Assignment 3 **docx** (CLI, zip `ex3_<id1>_<id2>.zip`, 5 folders, `students.txt`, plugins).
2. Skeleton headers in `common/` and `common_*` (as-is).
3. Common-issues PDF + Structuring PDF.
4. AdvCpp guideline + Error Code Key.
5. Ex2 code — ideas only.

**Submission Guidelines 2026B mixes Ex1.** It still says `drone_mapper`, `map_output.txt`, three
`original_output` folders, zip `ID1_ID2.zip`. Those are **not** Ex3 gaps. Compiler flags
(`g++ -std=c++20 -Wall -Wextra -Werror -pedantic`), HLD PDF, readme names/IDs, no binaries, and
forum-approved libraries **do** still apply.

`b*` in the Error Code Key are **deductions**, not bonuses. Real bonuses: `bonus.txt` + Assignment 3
optional rows / lazy `.so` / algorithm contest.

`Structuring the project.pdf` does **not** define `UserCommon/`, 4 build files, or registration
`.cpp` ownership — those are Assignment 3 docx rules.

## 5. Optional second wave — implementation audit

Only if the user asked to verify the **current tree** against the instructions. Dispatch **after**
catalogs exist (or in the same turn with catalogs pasted into the prompt). One agent per domain,
same turn:

| Agent | Scope |
|-------|--------|
| Structure / naming / CMake / frozen dirs | 5 folders, IDs, `PREFIX ""`, `ENABLE_EXPORTS`, `REGISTER_*` global, no `new`/`delete`, `git diff main -- common/ …` |
| CLI / plugins / threading / reports | Exact `-comparative` / `-competition`, output dirs, YAML keys, `num_threads`, `dlopen`/`dlclose`, `-verbose` plumbing |
| Algorithm + mandatory faults | CI1 + CI5 (MockMovement throw; who catches); skip optional PDF rows as deviations |

`subagent_type: explore`, **read-only**. Each prompt includes the relevant catalog IDs and “PASS/FAIL
with file:line; do not edit.”

Parent integrates: staff conflicts vs our deviations vs submission-time-only (HLD, zip). Write
pickup notes to `docs/assignment-compliance-pickup.md` if this is a standing queue. New shipped
gaps go through `populate-known-issues`.

## 6. What extractors must not miss (Assignment 3)

- CLI flags: **`-comparative`** and **`-competition`** (not `-competitive`).
- `num_threads` optional; `-verbose` is bracketed (treat optional). “All args mandatory” contradicts
  that — follow the optional bullets + brackets.
- Threading: missing/`=1` → main only; `>=2` → N workers **plus** main (total never 2).
- Registration `.cpp` in Simulator only; headers as-is.
- Comparative dir `comparative_results_<time>` under MC folder; competitive `competition_<time>`
  under algorithms folder.
- YAML keys exactly as in the docx (`comparative_report` / `competitive_report`, `same_results`,
  `errors`, …). Scoring formula and “agreeing” equality are **unspecified**.
- Final additional-notes item may still be `[TBD]`.

## 7. After extraction

- Diff catalogs against `docs/assignment3-checklist.md` / `docs/error-handling-matrix.md` /
  `docs/review-error-codes.md`. Update those condensations only if the user asked, or if a real
  staff change would mislead later agents.
- Do not implement Ex1 leftover folders from Submission Guidelines.
- Follow `docs/open-questions.md` working assumptions where the docx is silent.

## Additional resources

- Pickup / last audit queue: `docs/assignment-compliance-pickup.md`
- Known Issues rows: `.cursor/skills/populate-known-issues/SKILL.md`
- Zip / naming: `.cursor/skills/pre-submission-review/SKILL.md`
