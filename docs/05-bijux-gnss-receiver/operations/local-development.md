---
title: Local Development
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Local Development

When editing `bijux-gnss-receiver`, start from the owning runtime family, not
from the test name that happened to fail first.

## Good Local Loop

- find the owning family in `engine`, `pipeline`, `ports`, `artifacts`,
  `reference_validation`, `validation_report`, or `sim`
- update the crate-local docs if runtime meaning moves
- run targeted tests for that family before touching wider suites
- inspect `src/api.rs` if the change affects something public

## What To Avoid

- changing several stage families at once without naming the shared reason
- using one broad integration test as the only proof for a low-level runtime
  change
- widening exports to make local development easier

## Useful Local Anchors

- `crates/bijux-gnss-receiver/README.md`
- `crates/bijux-gnss-receiver/docs/`
- `crates/bijux-gnss-receiver/tests/`
