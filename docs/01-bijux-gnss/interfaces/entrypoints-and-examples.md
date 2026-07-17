---
title: Entrypoints And Examples
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# Entrypoints And Examples

Use this page to choose the right public starting point.

## Common Starting Points

- start from the `bijux` binary when the consumer is an operator or automation
- start from command contracts when the question is naming, flags, or workflow
  shape
- start from workflow or validation contracts when the question is which
  lower-owner behavior is being assembled
- start from facade contracts when the question is about Rust-package
  convenience rather than CLI execution

## Example Reader Routes

- "I need to know which command owns this workflow":
  command contracts, then workflow contracts
- "I need to know why this validation output looks this way":
  validation contracts, then reporting contracts
- "I need to know whether this Rust export belongs here":
  facade contracts

## Protecting Proof

Inspect `crates/bijux-gnss/docs/COMMANDS.md`,
`crates/bijux-gnss/docs/WORKFLOWS.md`,
`crates/bijux-gnss/docs/VALIDATION.md`, and `crates/bijux-gnss/src/lib.rs`.
Then inspect the matching CLI source under `crates/bijux-gnss/src/cli/` to
confirm these entry routes still reflect real public starting points.
