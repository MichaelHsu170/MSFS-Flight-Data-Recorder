---
name: review-loop
description: Runs the repository's review loop, rounds of fresh-reviewer review and fixes until clean, over a diff or, with "full", the whole project. Edits the working tree, so use it only when the user asks for a review loop or a full review, or invokes /review-loop; a plain request to review changes is one round of /diff-review.
---

# Review Loop

Read [`.github/code-review.md`](../../../.github/code-review.md) at the repository root first and follow its sections 1 and 3 exactly. It is the single source of truth for code review, shared with GitHub Copilot. You are the coordinator: each round's reviewer is a `diff-review` skill invocation with the brief as its arguments.

Arguments: $ARGUMENTS

`full` means a full review. A ref (commit/tag/branch) is a diff review's base. With neither, it's a diff review of uncommitted changes.
