---
mode: 'agent'
description: 'Run one round of code review (uncommitted changes, a named commit/tag/branch, or a review-loop brief) for bugs and suspicious behavior, with context and proposed fixes. Report-only: never edits the working tree.'
---

Read [.github/code-review.md](../code-review.md) first and follow its sections 1 and 2 exactly. It is the single source of truth for code review, shared with Claude Code. Run this in a new chat: its "review from zero known context" rule isn't met in a chat that wrote or discussed the changes.

Ref to check against, or a brief from the review loop (optional): ${input:ref:Commit/tag/branch to diff against, or a review-loop brief (leave blank to review only uncommitted changes)}

Do not edit, stage, or commit any files in the working tree during this review. Only read files, run the read-only `git` commands and the build commands listed in that file, and, for its coverage break-check, edit files in the throwaway copy under `build/review-copy/`. The output is a markdown report only, in the format that file specifies.
