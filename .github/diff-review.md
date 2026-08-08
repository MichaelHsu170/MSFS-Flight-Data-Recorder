# Diff Review — canonical process

This file is the single source of truth for the "diff-review" code-change review. Both entry points just point here and follow this exactly:

- Claude Code skill: [.claude/skills/diff-review/SKILL.md](../.claude/skills/diff-review/SKILL.md) (implicit invocation, or explicit `/diff-review`)
- GitHub Copilot: [.github/copilot-instructions.md](copilot-instructions.md) (implicit) and [.github/prompts/diff-review.prompt.md](prompts/diff-review.prompt.md) (explicit `/diff-review`)

If the review process needs to change, edit only this file — the other two are thin pointers and shouldn't drift from it.

**Hard rule: this is read-only. Never edit, stage, or commit any file while running this review — only read files and run read-only `git` commands (`git status`, `git diff`, `git log`, `git show`, `git rev-parse`, `git ls-files`, `git merge-base`). The report is the only output, including any fix proposals in it — they are text, not applied changes.**

**Hard rule: review from zero known context.** Do not rely on memory of any prior conversation about this diff — design rationale explained earlier in a chat session, reasoning already worked through with the author, prior confirmations that "this part is fine," etc. This review must be invocable cold (a fresh session, a different tool, a different reviewer entirely) and reach the same conclusions every time, so treat the diff and the current state of the repository as the *only* evidence available. Derive intent and correctness solely from the code, its comments, and the surrounding codebase. If a change's correctness depends on reasoning that isn't recoverable from the repo itself, that's a gap worth surfacing (e.g. as a missing-comment/context finding), not something to silently fill in from memory.

## 1. Determine scope

- If the user named a commit hash, tag, or branch to check against (in the request text, or as an argument after the command), resolve it first: `git rev-parse --verify <ref>`. If that fails, stop and say the ref doesn't resolve — don't guess or substitute another ref.
- If a ref was given, diff from that ref to the current working tree: `git diff <ref>`. This already includes any staged and unstaged uncommitted changes on top of that ref — nothing further is needed to include them.
- If no ref was given, review uncommitted changes only: `git diff HEAD` (covers both staged and unstaged modifications to tracked files).
- Either way, also check for new untracked files that aren't committed yet: `git status --porcelain`, lines starting with `??`. Treat their full content as part of the review (a diff against empty).
- If there is nothing to review (empty diff and no relevant untracked files), say so and stop. Do not review unrelated or unchanged code to fill the gap.

## 2. Review each change

- Read enough surrounding context (full function/file, callers, related types/structs) to understand what the changed code actually does — never judge a diff hunk in isolation.
- For each changed file, look for real defects introduced or exposed by the change: logic errors, off-by-one, inverted or incorrect conditionals, null/nullptr dereference, use-after-free, resource/memory leaks, race conditions and thread-safety issues, incorrect or missing error handling, type mismatches, sign/overflow issues, incorrect API usage, injection/unsanitized-input issues, inconsistent state updates, missed edge cases, and behavior that contradicts nearby comments or the evident intent of the change.
- Scope the review to the changed code and the code paths it affects — this is not a full audit of every file the diff happens to touch.
- Verify each candidate finding by tracing the actual code path before reporting it. If it can't be fully verified but is still worth flagging, mark it as such rather than dropping it or overstating it.
- Check that documentation stays in sync with the change:
  - Comments in and around the changed code must describe the *new* behavior, not the old one — search the repo for other comments, or `.md` docs (README, etc.), that still reference a name, field, or behavior this diff removed or changed, and flag any left stale.
  - If the change introduces a nontrivial new invariant, assumption, or design decision whose correctness isn't obvious from the code alone, and nothing in the repo explains it, flag that as a documentation gap (per the zero-known-context rule above — the next reader won't have this conversation to fall back on either).
  - This is about comments/docs actually touched or made stale by the diff, not a general documentation audit of unrelated code.
- Look for a parallel/sibling code path that mirrors the changed logic elsewhere (e.g. a twin branch for a related case — liftoff/touchdown, departure/destination, and similar paired paths are common in this codebase) and may need the same fix or update but didn't get it. Search for it explicitly rather than relying on having noticed it while reading the diff.
- If the change removes or renames a field, function, type, or flag, grep the whole repository (not just the changed files) for every remaining reference to the old name. A reference left behind is either a compile error or, worse, silently-dead code still reading/writing something nothing else maintains — treat any hit as a finding, not just a note.
- Look for dead code left behind by the change: a variable, parameter, field, branch, or helper function that the diff (or the state it leaves the code in) makes unreachable or unused, but that wasn't removed. This is distinct from the point above — it's not a broken reference to something deleted, it's code that still compiles and still runs but no longer does anything anyone can reach or that no longer serves the purpose its own comment claims. Flag it so it can be deleted rather than left to confuse the next reader.

## 3. Report format

Produce a single markdown report with two parts: a summary of changes, then the findings.

### Summary of changes

Before the findings, give a short, abstract, high-level description of what the diff does — a few sentences or a short bulleted list, grouped by theme (e.g. "renames X to Y across the schema/backend/frontend", "fixes wording in log messages", "adds a new field to track Z"). This is a plain-language orientation for the reader, not a file-by-file changelog and not a restatement of the diff — skip implementation detail, specific line numbers, and code snippets here; those belong in the findings section if relevant to a defect.

### Findings

Findings ordered most severe first. For each finding:

- **File / line**
- **Summary** — one sentence stating the defect
- **Context** — the relevant surrounding code or call path needed to see *why* it's a defect (a short quoted snippet or precise description of the state/flow involved), so the reader doesn't have to go re-derive it themselves
- **Failure scenario** — the concrete input or state that triggers it and what goes wrong as a result
- **Verdict** — `Confirmed` (traced and verified against the real code) or `Plausible` (suspicious, not fully verified)
- **Proposed fix** — a concrete suggestion: either a short code snippet/diff or a precise description of the change, specific enough that someone could implement it directly. This is a proposal only — it is never applied by this review.

If nothing was found, say so explicitly and note what was reviewed (files and scope), rather than leaving that ambiguous.

Do not fix, edit, stage, or commit anything as part of this review — the report, including its fix proposals, is text output only.
