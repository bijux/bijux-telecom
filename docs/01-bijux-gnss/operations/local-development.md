---
title: Local Development
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# Local Development

When editing `bijux-gnss`, start from the owning command family, not from the
lower crate that the command eventually invokes.

## Good Local Loop

- find the owning family in `src/cli/command_catalog/`, `src/cli/commands/`,
  `src/cli/command_runtime.rs`, `src/cli/command_runtime/`,
  `src/cli/command_support/`, `src/cli/report.rs`, or the facade
- update the crate-local docs if command meaning moves
- run targeted command tests before touching wider suites
- inspect `src/lib.rs` if the change affects the Rust facade

## What To Avoid

- changing several workflow families at once without naming the shared reason
- using one broad integration test as the only proof for a command-shape change
- widening facade exports to make local development easier

## Useful Local Anchors

- `crates/bijux-gnss/README.md`
- `crates/bijux-gnss/docs/`
- `crates/bijux-gnss/tests/`
