---
title: Local Development
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# Local Development

When editing `bijux-gnss-nav`, start from the scientific family, not from the
file that happened to fail first.

## Good Local Loop

- find the owning family in `src/formats/`, `src/orbits/`,
  `src/corrections/`, `src/estimation/position/`, `src/estimation/ppp/`,
  `src/estimation/rtk/`, `src/models/`, or `src/time.rs`
- update the crate-local docs if the scientific meaning moves
- run targeted tests for that family before touching wider suites
- inspect `src/api.rs` if the change affects something public

## What To Avoid

- changing several scientific families at once without naming the shared reason
- using one broad integration test as the only proof for a low-level change
- widening exports to make local development easier

## Useful Local Anchors

- `crates/bijux-gnss-nav/README.md`
- `crates/bijux-gnss-nav/docs/`
- `crates/bijux-gnss-nav/tests/`
