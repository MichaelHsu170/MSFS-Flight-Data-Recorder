# Repository custom instructions for GitHub Copilot

## Code quality rules

Follow these rules when writing or changing code. This section is the single source of truth for them, shared with Claude Code (`CLAUDE.md` imports this file) and checked by the diff review ([.github/diff-review.md](diff-review.md)). If a rule needs to change, edit only this section.

- **Minimal**: write only the code the change needs. No speculative hooks, unused methods or parameters, or "just in case" branches. Code made dead by a change is deleted in the same change.
- **Modular**: each concern has one owner. Callers use the owner's API instead of reimplementing its logic.
- **Reuse first**: before writing a helper, search for an existing one and extend it. Never add a second implementation of the same logic (a second error-code table, a second copy of a loop).
- **Consolidate what you touch**: when a change meets duplicated logic, extract one shared helper rather than adding another copy.
- **Evidence-based conclusions**: a claim that work is complete, fixed, or compliant cites the evidence that proves it — the check that was run (command, scan, test, on-device step) and its actual output, with file:line references. No conclusion from memory or assumption; a check that was not run is reported as not verified.
- **Docs always up to date**: a code change that alters behavior, structure, or a documented decision updates the matching docs/docstring in the same change. A change is not complete while its spec describes the old code.
- **Test coverage as high as possible**: a code change adds or updates tests covering the new or changed behavior, including edge cases and error paths, to the highest coverage practically achievable. Code that cannot be unit-tested (e.g. requires live SimConnect/hardware) is covered as far down the boundary as possible, and the untestable remainder is noted in the PR description along with why.
- **Tests verify, not reimplement**: a test asserts against an independently known expected result (fixed literals, hand-computed values, fixture/golden data, documented spec behavior), never by recomputing the same formula or logic the implementation uses. A test that would still pass after the implementation's logic is broken in the same way is not a valid test.
- **Builds warning-clean**: a change compiles with no new compiler warnings. An existing warning touched by the change is fixed, not just preserved; a warning that cannot be fixed is suppressed narrowly (specific warning code, smallest scope) with a comment explaining why.

## Code change review

When asked to review, check, audit, or sanity-check code changes, a diff, recent edits, or uncommitted changes for bugs, incorrect logic, or suspicious behavior — whether asked directly in chat or via `/diff-review` — read [.github/diff-review.md](diff-review.md) and follow it exactly. It is the single source of truth for this review (scope rules, methodology, report format, and required fix proposals), shared with the Claude Code version of the same review (`.claude/skills/diff-review/SKILL.md`).

Read-only: never edit, stage, or commit files as part of this review — the report, including any fix proposals in it, is text output only.
