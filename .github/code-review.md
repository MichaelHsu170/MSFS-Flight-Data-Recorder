# Code review: canonical process

This file is the single source of truth for code review in this repository: what a review checks, how one round runs, how rounds form a loop, and this project's modules, break check and live-check limits. The entry points below only point here. If the review process needs to change, edit only this file.

| Review | What it does | Claude Code | GitHub Copilot |
|---|---|---|---|
| One round | Reviews uncommitted changes or the diff against a ref. Report only; never fixes. | `/diff-review [ref]` ([skill](../.claude/skills/diff-review/SKILL.md)) | `/diff-review` ([prompt](prompts/diff-review.prompt.md)), in a new chat |
| Diff review loop | Rounds of review and fixes over a diff until clean. Edits the working tree. | `/review-loop [ref]` ([skill](../.claude/skills/review-loop/SKILL.md)) | `/review-loop` ([prompt](prompts/review-loop.prompt.md)) |
| Full review loop | The same, starting from the whole project, module by module. | `/review-loop full` | `/review-loop`, with `full` as its input |

A request to review, check, audit or sanity-check changes is one round. Only a request for the loop or for a full review starts the loop, because only the loop changes code.

Section 1 applies to everyone. A reviewer follows section 2; whoever runs the loop (the coordinator) follows section 3. Section 4 records decisions not to fix.

## 1. Rules for every review

### 1.1 How a reviewer works

**Hard rule: the working tree is read-only for a reviewer. Never edit, stage, or commit any file in it while reviewing. Only read files, run read-only `git` commands (`git status`, `git diff`, `git log`, `git show`, `git rev-parse`, `git ls-files`, `git merge-base`), run the build-and-test command `cmd /c ".vscode\scripts\build.bat Debug test rebuild"`, and, for the coverage break-check (section 2.3), run `powershell -NoProfile -ExecutionPolicy Bypass -File .vscode\scripts\review_copy.ps1`, run the copy's own `build.bat`, and edit files under `build\review-copy\`. All of these write only to the git-ignored `build/` directory: build output, and `build\review-copy\`, a throwaway copy of the working tree that is the only place a reviewer edits a file. The report is the only output, including any fix proposals in it; they are text, not applied changes.**

**Hard rule: review from zero known context.** Do not rely on memory of any prior conversation about this diff: design rationale explained earlier in a chat session, reasoning already worked through with the author, prior confirmations that "this part is fine," etc. This review must be invocable cold (a fresh session, a different tool, a different reviewer entirely) and reach the same conclusions every time, so treat the diff and the current state of the repository as the *only* evidence available. Derive intent and correctness solely from the code, its comments, and the surrounding codebase. If a change's correctness depends on reasoning that isn't recoverable from the repo itself, that's a gap worth surfacing (as a nit, section 1.2), not something to silently fill in from memory. Re-reading the files in the session that wrote or discussed them doesn't meet this rule, so each entry point starts the review without that conversation: the Claude Code skill runs in a forked subagent, and in Copilot `/diff-review` is run in a new chat.

### 1.2 What counts as a finding

A **finding** is one of these, each with a concrete failure scenario:

- `Bug`: a defect in the code, including a build or test that fails.
- `False statement`: a doc or comment that says something the code doesn't do, including a stale "Not covered" or section 4 entry. Its scenario is what a reader relying on it would get wrong.
- `Test`: an invalid test, or a missing test for promised behavior, as the code quality rules define them. Its scenario is the break that goes unnoticed; a break no test catches (section 2.3) is one.

A **nit** is anything else section 2.2 or a code quality rule flags with no failure scenario: dead code, a duplicated helper, an unneeded parameter, a new compiler warning, an undocumented assumption. Report it with kind `Nit`. The loop's coordinator fixes it, but it never keeps the loop going: a round with only nits counts as clean.

A plausible finding is verified before it counts. The reviewer traces it as far as it can and reports it as `Plausible` if that doesn't settle it. In the loop, the coordinator then traces or reproduces it: it becomes `Confirmed`, or it is dropped and listed in the section 3.6 report with the trace that refuted it. One the coordinator can't settle goes to the owner.

These are **not findings**:

- Code listed under "Not covered" in [tests/README.md](../tests/README.md), or a decision in section 4, unless the entry has become false.
- A defect the diff neither introduces nor exposes. It goes in the report's "Outside scope" list, and the owner decides what to do with it.
- A way in that can't happen in the running app: an allocator failure, a hand-edited database, a simulator breaking the SDK's documented behavior.
- A test that would only pin an incidental detail or kill a mutant, which the code quality rules forbid.
- A style preference no rule states, a refactoring idea, or a performance worry with no concrete symptom.
- A `tst_map_widget` timeout during another test run or while the desktop was in use. Rerun it alone first; it counts if it fails again.

### 1.3 Live checks

The automated tests never touch the simulator, the AI service or the owner's data. These checks do, or get in the owner's way:

| Check | Why it's unsafe | When it may run |
|---|---|---|
| Launching the app | A Debug build uses the files in its working directory, and the Debug launch configuration starts it in the repository root, where the owner's real `flight_data.db`, `settings.ini` and log are. A Release build uses the ones next to its exe. On start the app migrates the database, which can't be undone. A running copy also blocks a second one. | Never by a reviewer. The coordinator only with a Debug build in an empty temporary working directory, and only when no test can show the behavior. |
| A flight in MSFS through the real SimConnect | It needs the owner in the simulator, and it writes real trips. | Only by the owner. The "To check" steps are under "Not covered" in tests/README.md. |
| The real AI service | It uses the owner's `gemini_api_key` and quota, and sends flight data out. | Only by the owner. Nobody reads or prints that key; the tests stub `fetch()`. |
| The test suite | Safe for data, but the map tests show real windows, move the real mouse and use the clipboard, so using the desktop during a run can fail them. | One run at a time (tests/README.md), in the foreground. |

A check that can't run here is reported as not verified, and the final report tells the owner what to check by hand.

## 2. One round

A round is one fresh reviewer, one scope and one report. `/diff-review` is a single round; the loop runs several.

A full review's first round has no diff. Wherever this section or section 1.2 says "the change" or "the diff", it means the module's code as it stands, and test coverage is judged by reading the tests, plus the breaks chosen for section 3.1's step 4.

### 2.1 Determine scope

- If the request is a brief from the review loop (section 3.4), it sets the scope: the named snapshots' `git diff <from> <to>` and the named files, or for a full review's first round the named module read as a whole. Skip the rest of this list.
- If the user named a commit hash, tag, or branch to check against (in the request text, or as an argument after the command), resolve it first: `git rev-parse --verify <ref>`. If that fails, stop and say the ref doesn't resolve; don't guess or substitute another ref.
- If a ref was given, diff from that ref to the current working tree: `git diff <ref>`. This already includes any staged and unstaged uncommitted changes on top of that ref, so nothing further is needed to include them.
- If no ref was given, review uncommitted changes only: `git diff HEAD` (covers both staged and unstaged modifications to tracked files).
- Either way, also check for new untracked files that aren't committed yet: `git status --porcelain`, lines starting with `??`. Treat their full content as part of the review (a diff against empty).
- If there is nothing to review (empty diff and no relevant untracked files), say so and stop. Do not review unrelated or unchanged code to fill the gap.

### 2.2 Review each change

- Read enough surrounding context (full function/file, callers, related types/structs) to understand what the changed code actually does; never judge a diff hunk in isolation.
- For each changed file, look for real defects introduced or exposed by the change: logic errors, off-by-one, inverted or incorrect conditionals, null/nullptr dereference, use-after-free, resource/memory leaks, race conditions and thread-safety issues, incorrect or missing error handling, type mismatches, sign/overflow issues, incorrect API usage, injection/unsanitized-input issues, inconsistent state updates, missed edge cases, and behavior that contradicts nearby comments or the evident intent of the change.
- Scope the review to the changed code and the code paths it affects; this is not a full audit of every file the diff happens to touch.
- Verify each candidate finding by tracing the actual code path before reporting it. If it can't be fully verified but is still worth flagging, mark it as such rather than dropping it or overstating it.
- Check that documentation stays in sync with the change:
  - Comments in and around the changed code must describe the *new* behavior, not the old one. Search the repo for other comments, or `.md` docs (README, etc.), that still reference a name, field, or behavior this diff removed or changed, and flag any left stale.
  - If the change introduces a nontrivial new invariant, assumption, or design decision whose correctness isn't obvious from the code alone, and nothing in the repo explains it, flag that as a documentation gap (per the zero-known-context rule above: the next reader won't have this conversation to fall back on either).
  - This is about comments/docs actually touched or made stale by the diff, not a general documentation audit of unrelated code.
- Look for a parallel/sibling code path that mirrors the changed logic elsewhere (e.g. a twin branch for a related case; liftoff/touchdown, departure/destination, and similar paired paths are common in this codebase) and may need the same fix or update but didn't get it. Search for it explicitly rather than relying on having noticed it while reading the diff.
- If the change removes or renames a field, function, type, or flag, grep the whole repository (not just the changed files) for every remaining reference to the old name. A reference left behind is either a compile error or, worse, silently-dead code still reading/writing something nothing else maintains. Treat any hit as a finding, not just a note.
- Look for dead code left behind by the change: a variable, parameter, field, branch, or helper function that the diff (or the state it leaves the code in) makes unreachable or unused, but that wasn't removed. This is distinct from the point above: it's not a broken reference to something deleted, it's code that still compiles and still runs but no longer does anything anyone can reach or that no longer serves the purpose its own comment claims. Flag it so it can be deleted rather than left to confuse the next reader.

### 2.3 Check code quality compliance

Check the change against every rule in the "Code quality rules" section of [.github/copilot-instructions.md](copilot-instructions.md), the single source of truth for them. Read it fresh rather than working from a remembered list, and take each rule there one by one, including any added since this file was written. Each rule gets a verdict in the report (see section 2.4) backed by evidence, never by assumption:

- Rules judged by reading the diff (e.g. minimal, modular, reuse first, consolidate, docs up to date): search the repository for what the rule is about (an existing helper the change duplicates, a parameter or branch nothing uses, a doc still describing the old behavior) and cite the file:line or the search that showed it.
- Tests check promised behavior: for each new or changed test, name the production class and the promised behavior it checks, and confirm three things:
  - it asserts an observable result;
  - its expected values are fixed literals, hand-computed values, fixtures or documented behavior, not the implementation's own formula recomputed;
  - it isn't one of the rule's invalid forms (only calling code with no observable effect, asserting what the test itself set up, pinning an incidental detail).
- Test coverage: for each behavior the change adds or alters, find the valid test that covers it and read its assertions to judge whether the test would fail if that code broke. Changed code with no such test that isn't listed, with its reason, under "Not covered" in [tests/README.md](../tests/README.md) is a violation. Reading only suggests a test would fail; the rule counts a test only once breaking its code has made it fail, so then prove it with the break-check:
  - When: only once everything else in sections 2.2 and 2.3 has no finding (Confirmed or Plausible), or when the request asks for the break-check. In the loop, a round's brief leaves it out and a separate brief asks for it (section 3.1). It rebuilds and reruns tests for every break, so it takes long (roughly half a minute to a minute per break), and every fix made for a finding changes the code it would check. Skipped, the verdict is "Not verified", saying it runs once the review has no other findings, with what reading the tests showed as the evidence.
  - Set up the copy: `powershell -NoProfile -ExecutionPolicy Bypass -File .vscode\scripts\review_copy.ps1` syncs `build\review-copy\` to the working tree. Then build it and run every test: `cmd /c "build\review-copy\.vscode\scripts\build.bat Debug test"` (the first time, this configures and builds from scratch; allow it 10 minutes). Every test must pass before anything is broken, or a failure proves nothing; if it can't be built or a test fails, the verdict is "Not verified".
  - List what to break: every condition, computed value, constant, stored, sent or shown field, branch and error path the change adds or alters in production code (C++, `map.html` and `charts_panel.qml`; not tests or docs).
  - Break each one in the copy, one at a time: a small edit to a file under `build\review-copy\` that compiles and changes what the code does (invert or drop a condition, change a constant or an offset, drop a statement, swap two fields, rename a key). Build and run the test programs that should catch it: `cmd /c "build\review-copy\.vscode\scripts\build.bat Debug only <test> [<test>...]"` (caught: it builds, then ctest reports the test failed, `The following tests FAILED`, with a nonzero exit code; every test passed before the break, so the break caused it. A test program's own `FAIL!` lines may not show, since its output isn't always captured. A compile error, `error C` or `error LNK` with no `Testing` line, proves nothing, so make another break). Then undo it before the next break: `powershell -NoProfile -ExecutionPolicy Bypass -File .vscode\scripts\review_copy.ps1 <file>` (the file's path from the repository root).
  - After undoing the last break, build the copy and run every test again: `cmd /c "build\review-copy\.vscode\scripts\build.bat Debug test"`. Undoing a break restores only the source, so the copy's test programs would otherwise still be the ones built from the last break; the copy's `coverage.ps1` runs whatever is built, so its next run would report that break's failures. Every test must pass, as before the first break.
  - A break no test catches is a violation, reported as a finding whose proposed fix is the test that would catch it, unless "Not covered" in tests/README.md lists that code with the reason no valid test can catch it (or that it needs the real SimConnect or hardware).
  - Evidence: per file, the number of breaks made and caught; each break not caught, with what it changed and its "Not covered" entry or finding.
  - Run one build or test run at a time, in the foreground: two test runs at once can make the map tests time out (tests/README.md).
- Builds warning-clean: if a loop's brief gives this round's build output, use it as the evidence and don't build again; if it says only docs changed, the verdict is "Not verified". Otherwise run `cmd /c ".vscode\scripts\build.bat Debug test rebuild"` (`rebuild` recompiles every file, so warnings from files an earlier build already compiled show too; it takes several minutes, so allow it a 10-minute timeout) and search its output for `warning C`, `error C` and `LNK4`, and for the ctest summary line. Cite the exit code, the match count and the summary line. If the command can't be run, the verdict is "Not verified", not "Meets".

A violation is also reported under Findings (section 2.4) with the usual fields, and the kind section 1.2 gives it.

### 2.4 Report format

Produce a single markdown report with three parts: a summary of changes, the code quality compliance check, then the findings.

#### Summary of changes

Before the findings, give a short, abstract, high-level description of what the diff does: a few sentences or a short bulleted list, grouped by theme (e.g. "renames X to Y across the schema/backend/frontend", "fixes wording in log messages", "adds a new field to track Z"). This is a plain-language orientation for the reader, not a file-by-file changelog and not a restatement of the diff. Skip implementation detail, specific line numbers, and code snippets here; those belong in the findings section if relevant to a defect.

When the scope spans multiple commits (a ref diff, not just uncommitted changes), describe only the net result of all of them together: what the code does now versus at the ref, not a commit-by-commit narration of how it got there. Don't structure this section around individual commits or cite commit hashes/messages; treat the whole range as one diff.

#### Code quality compliance

A table with one row per rule from section 2.3, with three columns: the rule's name; its verdict, which is `Meets`, `Violation` (naming the finding that describes it) or `Not verified` (saying what wasn't run or couldn't be checked); and the evidence: the command and its actual output, the search, or the file:line references that support the verdict.

#### Findings

Findings ordered most severe first. For each finding:

- **File / line**
- **Summary**: one sentence stating the defect
- **Context**: the relevant surrounding code or call path needed to see *why* it's a defect (a short quoted snippet or precise description of the state/flow involved), so the reader doesn't have to go re-derive it themselves
- **Kind**: `Bug`, `False statement`, `Test` or `Nit` (section 1.2)
- **Failure scenario**: the concrete input or state that triggers it and what goes wrong as a result (`None` for a nit)
- **Verdict**: `Confirmed` (traced and verified against the real code) or `Plausible` (suspicious, not fully verified)
- **Proposed fix**: a concrete suggestion: either a short code snippet/diff or a precise description of the change, specific enough that someone could implement it directly. This is a proposal only; it is never applied by this review.

If nothing was found, say so explicitly and note what was reviewed (files and scope), rather than leaving that ambiguous. A defect noticed outside the scope goes in a short "Outside scope" list after the findings.

## 3. The loop

The coordinator runs the rounds, fixes the findings and decides what happens next. It may be the session that wrote the code, because reviewers judge the code; the coordinator only verifies what they report, and its report shows anything it dropped. Reviewers get the code and the rules but not the coordinator's conclusions, and they report rather than fix.

### 3.1 Flow

1. **First round.** Build and take the first snapshot (section 3.3). For a diff review, resolve the base once with `git rev-parse` (the ref, or `HEAD` with none) and use that hash in every later diff; the scope is all the diffs, from the base to the snapshot. If that diff is empty, say so and stop. For a full review, the scope is the whole project, with one reviewer per module (section 3.2).
2. **Fix** every finding and nit (section 3.5). If the round had no findings, even if it had nits, go to step 4. A finding from any round that needs the owner's decision or a live check isn't fixed or counted again: ask the owner and fix the rest. If it's the only finding, wait for the answer. The loop can't stop with one unanswered, except at the limit.
3. **Next round.** It covers the previous round's diffs plus what they touch: callers, sibling paths, and the docs and tests that describe them. Measure that scope again from the new snapshot; never reuse the last list. Go to step 2.
4. **Break check.** After a clean round, one fresh reviewer runs only the break-check in section 2.3. A survivor is a finding: fix it and go to step 3. With no survivors, go to step 5. If the check comes back "Not verified", fix the cause and run it again; if it still can't run, go to step 5 and report it as not verified.
   - A diff review breaks all the code the loop's diffs add or alter. Coming back here later, it breaks what changed since the last break check and repeats every break that survived.
   - A full review breaks the same, plus up to 10 breaks per module, chosen by that module's first-round reviewer as the conditions and error paths whose tests it trusts least and listed in its report. Breaking the whole project isn't practical at up to a minute per break.
5. **Final round.** One fresh reviewer covers all the diffs of the loop, with the first of these focuses that fits its diffs and this loop hasn't used:
   - callers and real tool behavior, if production code or scripts changed: each changed call against its callers and what the SimConnect SDK, Qt and SQLite headers and docs on this machine actually say (anything they don't settle, such as the Gemini service, is reported as not verified);
   - test validity, if tests changed: each changed test against the code quality rules;
   - docs against code: README.md, the tests/README.md Coverage table, settings and comments. If no focus is left, repeat this one.

   A diff review's diffs run from its base to the latest snapshot; a full review's run from its first snapshot. A full review that changed nothing has no diffs, so it stops after the break check.
6. **Finish or go back.** If the final round has no findings, fix any nits and stop. If it has findings, fix them and go to step 3.

**Stop and report to the owner** after two failed final rounds, or when the sixth round or anything after it has findings (final rounds count; a full review's first round counts as one; the break check doesn't count, but its survivors are findings). A clean sixth round still goes on to the break check and a final round. That many findings usually means a design problem that more review won't fix, so don't start another round or fix the last round's findings: report them with their proposed fixes.

### 3.2 Modules and reviewers

A full review's first round has one reviewer per module. Each reads its production code, its tests and the docs that describe them, and looks outside the module only to follow a call.

| Module | Production code (`MSFS-Flight-Data-Recorder/`) | Tests (`tests/`) and docs |
|---|---|---|
| Recording | `recording/` | `recording/`, `fake_simconnect.*`, `test_support.*`; README.md's SimConnect section |
| Database and analysis | `db/` | `db/`; README.md's Database section |
| App and export | `app/`, `export/` | `app/`, `export/`; README.md's settings.ini Reference, Runtime Files and Export and Images sections |
| Panels | `ui/*` (not its subfolders) | `ui/*`; README.md's Trip Groups section |
| Charts | `ui/charts/` | `ui/charts/`, `local_time_zone.h` |
| Map and AI analysis | `ui/map/` | `ui/map/`, `data/`; README.md's AI Liftoff & Landing Analysis section |
| Build, tooling and instructions | none; `CMakeLists.txt`, `CLAUDE.md`, `.gitignore`, `.gitattributes`, `.vscode/`, `.github/`, `.claude/` | `tests/CMakeLists.txt`, the running and how-they-work text in tests/README.md; README.md's build sections, Tests, Output Locations and Project Structure |

**Starting a reviewer.** Every reviewer starts with none of the conversation:

- Claude Code: invoke the `diff-review` skill with the brief as its arguments. It forks without the conversation and pre-approves a reviewer's tools; section 1.1 is what keeps it read-only. Don't use a subagent that inherits the conversation.
- Copilot: the coordinator prints the brief, and the owner runs `/diff-review` with it in a new chat and pastes the report back.

Module reviewers may run at once, but two test runs may not (tests/README.md), and a test run needs the build it runs, so neither may two builds. So the coordinator runs each round's build and briefs tell reviewers not to build; only the break-check reviewer builds, and it runs alone.

### 3.3 Measuring rounds

The loop doesn't commit to measure its rounds. Before each round and each break check, in this order:

1. Before a round only (a break check builds its own copy), run `cmd /c ".vscode\scripts\build.bat Debug test rebuild"` in the foreground, read as section 2.3 says, unless only docs changed since the last one. Fix any failure.
2. Run `powershell -NoProfile -ExecutionPolicy Bypass -File .vscode\scripts\review_snapshot.ps1`. It prints a tree hash of the working tree, new files included, without touching the index or HEAD. `git diff <from> <to>` between two hashes is a round's scope.
3. Edit nothing until the reviewer has reported, so the code it reads is the code the snapshot names.

### 3.4 Briefs

A brief is the only context a reviewer gets. Narrow scope alone saves little, so keep the whole brief as narrow as the round:

```text
Follow .github/code-review.md sections 1 and 2. This round's scope: git diff <from> <to>.
Also read: <the exact callers, sibling paths, docs and tests measured for this round>.
Questions: <a few specific, neutral questions, e.g. "Can X be empty when Y calls it?">
Focus (final round only): <one focus from section 3.1>.
Checks already run: <command, exit code, ctest summary and warning count, or "only docs changed">.
Don't build. Don't run the break-check.
```

Leave out why a fix is right, what the previous round found, and any "this part is fine". A full review's first-round brief names the module instead of a diff, and asks for its chosen breaks. A break-check brief says "Run only the break-check in section 2.3" and names the snapshots and any chosen breaks.

### 3.5 Fixes and commits

- The coordinator fixes between rounds, following the code quality rules, with tests and docs in the same fix, and runs the tests behind the fix.
- A confirmed bug stays open unless the owner decides otherwise, recorded in section 4. Code no valid test can catch goes under "Not covered", this repository's list of what the code quality rules require to be listed as untested.
- The loop never commits, stages or pushes on its own. Fixes stay uncommitted in the working tree, and the owner commits them or asks for that once the loop has stopped. A commit the owner makes mid-loop is fine: snapshots don't depend on commits.

### 3.6 Report to the owner

When the loop stops at the end or at the limit, report: how it ended; each round's scope and result, with its findings and fixes; the checks run with their actual output; anything recorded in section 4 or "Not covered"; plausible findings dropped, with the trace that refuted them; nit fixes no reviewer has seen; "Outside scope" defects; questions still open for the owner; and what wasn't verified, including live checks left for the owner. After a stop at the limit, also say what the findings have in common and what design problem they suggest.

## 4. Decisions not to fix

A finding the owner decides not to fix is recorded here, where later reviewers read it: the file and behavior, why it stays, and the date. Code no valid test can catch goes under "Not covered" in tests/README.md instead.

None yet.
