---
name: diff-review
description: Reviews code changes for bugs, incorrect logic, and suspicious behavior — uncommitted working-tree changes by default, or the diff against a commit/tag/branch the user names (still including any uncommitted changes on top of that ref). Produces a findings report with context and proposed fixes; never edits, stages, or commits anything. Use whenever the user asks to review, check, audit, or sanity-check a diff, recent edits, uncommitted changes, or changes against a ref for bugs, regressions, or suspicious behavior.
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
---

# Diff Review

Read [`.github/diff-review.md`](../../../.github/diff-review.md) at the repository root first — it is the single source of truth for this review's scope rules, methodology, and report format (shared with the GitHub Copilot version of this same review). Follow it exactly, including the required Context and Proposed fix fields in the report.

If a ref (commit/tag/branch) was passed as an argument to `/diff-review` or named in the request, that's the ref referred to in that file's scope-determination step; if none was given, follow its no-ref path (uncommitted changes only).

This skill's tool access is already locked to read-only file tools and read-only `git` commands (see `allowed-tools` above). Never attempt to Edit, Write, or run a mutating `git`/shell command during this skill, even if asked to "just fix it" mid-review — applying a fix is a separate, explicit follow-up outside this skill's scope.
