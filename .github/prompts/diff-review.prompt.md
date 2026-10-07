---
mode: 'agent'
description: 'Review code changes (uncommitted, or against a named commit/tag/branch) for bugs and suspicious behavior, with context and proposed fixes. Report-only: do not edit any files.'
---

Read [.github/diff-review.md](../diff-review.md) first. It is the single source of truth for this review (scope rules, methodology, code quality compliance check, and report format, including the required Context and Proposed fix in each finding), shared with the Claude Code version of this same review (`.claude/skills/diff-review/SKILL.md`). Follow it exactly. Run it in a new chat: its "review from zero known context" rule isn't met in a chat that wrote or discussed the changes.

Ref to check against (optional): ${input:ref:Commit/tag/branch to diff against (leave blank to review only uncommitted changes)}

Do not edit, stage, or commit any files during this review. Only read files and run the read-only `git` commands and the build-and-test command listed in that file. The output is a markdown report only, in the format that file specifies.
