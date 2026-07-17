---
title: Release And Versioning
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# Release And Versioning

Command release notes should explain public workflow and output movement, not
only file movement.

## Release Notes Should Mention

- which command or workflow family changed
- whether the change affects public binary behavior, only internal wiring, or
  both
- whether reporting or validation publication changed
- whether downstream Rust users need to revisit facade assumptions

## Versioning Pressure Points

- new command names or flags
- changed operator-facing output meaning
- changed validation or report publication paths
- changed facade exports in `src/lib.rs`

## First Proof Check

Inspect `crates/bijux-gnss/docs/COMMANDS.md`,
`crates/bijux-gnss/docs/REPORTING.md`,
`crates/bijux-gnss/docs/VALIDATION.md`, and
`crates/bijux-gnss/src/lib.rs`. Then inspect
`crates/bijux-gnss/src/cli/` and the most relevant workflow integration tests
to confirm that any claimed release safety still matches real command
semantics, output meaning, and facade behavior.
