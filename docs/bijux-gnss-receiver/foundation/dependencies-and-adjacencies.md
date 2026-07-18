---
title: Dependencies And Adjacencies
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Dependencies And Adjacencies

This page explains which nearby crates `bijux-gnss-receiver` depends on
directly and which ones sit beside it as boundary checks.

## Direct Technical Dependencies

- `bijux-gnss-core` for shared contracts and runtime artifact vocabulary
- `bijux-gnss-signal` for signal primitives and lower-level reusable
  acquisition and tracking inputs
- `bijux-gnss-nav` for navigation-stage and validation-adjacent behavior when
  the `nav` feature is enabled

## Adjacencies That Matter More Than The Cargo Graph

- `bijux-gnss` is the main consumer whose operator pressure can tempt command
  policy into the runtime
- `bijux-gnss-infra` is the main persistence neighbor whose repository
  responsibilities can tempt file-layout logic into runtime artifacts
- `bijux-gnss-testkit` can supply truth and scenario support, but should not
  become the owner of runtime composition

## Practical Reading Rule

If a change needs synchronized edits in receiver and a neighboring crate,
review the boundary first. Cross-crate edits often signal unclear ownership
even when the code still compiles.
