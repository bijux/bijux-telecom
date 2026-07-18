---
title: Definition Of Done
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# Definition Of Done

A command change is done when a reader can run or inspect the command path,
understand which lower crate owns the real behavior, and see proof for the
operator-facing claim. The command crate is allowed to compose; it is not
allowed to hide ownership.

## Completion Gate

| changed surface | done means | proof to start from |
| --- | --- | --- |
| command name or flags | the public invocation is documented and stable enough for operators | [command guide](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss/docs/COMMANDS.md) plus a focused CLI integration test |
| workflow sequencing | the command still hands work to the owning lower crate in a readable order | [workflow guide](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss/docs/WORKFLOWS.md) plus the integration test for that workflow |
| reporting | output explains the lower-crate result without inventing new scientific or repository meaning | [reporting guide](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss/docs/REPORTING.md) plus report assertions |
| validation command | accepted and rejected cases identify the owner of the failed contract | [validation guide](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss/docs/VALIDATION.md) plus validation integration proof |
| Rust facade | exports remain a deliberate convenience layer, not an accidental mixed library | [facade guide](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss/docs/FACADE.md), [public API](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss/docs/PUBLIC_API.md), and guardrail tests |

## Reader Questions Before Commit

- What operator problem does this command surface solve?
- Which lower crate owns the behavior after argument parsing?
- What exact output, exit status, artifact, or validation result changed?
- Which test would fail if the command handed work to the wrong owner?
- Does the handbook route the reader to the owning crate when the question is
  no longer command-level?

## Proof Route

1. Read the [command boundary guide](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss/docs/BOUNDARY.md).
2. Read the relevant command-local guide: command, workflow, reporting,
   validation, or facade.
3. Select the focused test from the [command test guide](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss/docs/TESTS.md).
4. Update this handbook only when the reader-facing command contract moved.

Do not call a command change done because a lower crate test passed. The
command proof must show that the operator route still selects, presents, and
explains the lower-owned behavior correctly.
