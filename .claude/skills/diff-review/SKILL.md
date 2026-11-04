---
name: diff-review
description: Runs one round of code review for bugs, incorrect logic, and suspicious behavior, over uncommitted working-tree changes (the default), the diff against a commit/tag/branch the user names (still including any uncommitted changes on top of that ref), or a brief from the review loop. Produces a findings report with context and proposed fixes; never edits, stages, or commits anything in the working tree (its coverage break-check edits only a throwaway copy under build/). Use whenever the user asks to review, check, audit, or sanity-check a diff, recent edits, uncommitted changes, or changes against a ref for bugs, regressions, or suspicious behavior.
context: fork
allowed-tools:
  - Read
  - Grep
  - Glob
  - Bash(git status *)
  - Bash(git diff *)
  - Bash(git log *)
  - Bash(git show *)
  - Bash(git rev-parse *)
  - Bash(git ls-files *)
  - Bash(git merge-base *)
  - PowerShell(cmd /c ".vscode\scripts\build.bat Debug test rebuild")
  - PowerShell(powershell -NoProfile -ExecutionPolicy Bypass -File .vscode\scripts\review_copy.ps1*)
  - PowerShell(cmd /c "build\review-copy\.vscode\scripts\build.bat Debug *")
  - Edit(build/review-copy/**)
---

# Diff Review

Read [`.github/code-review.md`](../../../.github/code-review.md) at the repository root first and follow its sections 1 and 2 exactly. It is the single source of truth for code review, shared with GitHub Copilot.

This skill runs in a forked subagent (`context: fork` above) that starts without the conversation it was invoked from, which is how it meets that file's "review from zero known context" rule: the reviewer can't lean on design rationale or earlier "this part is fine" conclusions from the chat, even when the same session wrote the code.

Run every build (each `build.bat` command) in the foreground with a 600000 ms timeout, never in the background: this review runs in a forked subagent, and a background build is killed as soon as the subagent ends its turn.

Arguments: $ARGUMENTS

If a ref (commit/tag/branch) was given in the arguments above, that's the ref referred to in that file's scope-determination step; if a review-loop brief was given, it sets the scope; if neither was given, follow its no-ref path (uncommitted changes only).

This skill pre-approves read-only file tools, read-only `git` commands, the build-and-test command the compliance check runs, and, for the coverage break-check, the copy script, the copy's `build.bat`, and Edit on files under `build/review-copy/`; all of it writes only to the git-ignored `build/` directory (see `allowed-tools` above). Pre-approving doesn't block other tools, so the rule that follows is what keeps this review read-only. Never Edit a file outside `build/review-copy/`, Write any file, or run a mutating `git`/shell command during this skill, even if asked to "just fix it" mid-review. Applying a fix is a separate, explicit follow-up outside this skill's scope.
