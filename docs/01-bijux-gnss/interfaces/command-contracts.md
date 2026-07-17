---
title: Command Contracts
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# Command Contracts

These contracts define the stable operator-facing command shape.

## Owned Command Surfaces

- command families and subcommands, especially artifact, ingest, synthetic,
  validate, analyze, and diagnostics routes
- stable argument and flag shape
- operator-facing success and failure routing
- top-level workflow selection across lower-level crates

## Stability Expectations

- command names and flag meaning should be reviewable as durable public surface
- workflow composition may evolve, but the command layer must remain honest
  about what changed
- lower-owner science should remain behind command orchestration rather than
  being redefined here

## Closest Proof

- `crates/bijux-gnss/src/cli/command_catalog/`
- `crates/bijux-gnss/src/cli/command_line.rs`
- `crates/bijux-gnss/src/cli/commands/`
- `crates/bijux-gnss/docs/COMMANDS.md`
