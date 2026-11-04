---
mode: 'agent'
description: 'Run the review loop: rounds of fresh-reviewer review and fixes until clean, over a diff or the whole project. Edits the working tree.'
---

Read [.github/code-review.md](../code-review.md) first and follow its sections 1 and 3 exactly. It is the single source of truth for code review, shared with Claude Code. You are the coordinator: start each round's reviewer as its section 3.2 says.

Scope: ${input:scope:"full" for the whole project, a commit/tag/branch as a diff's base, or blank for uncommitted changes}
