---
title: Dependencies And Adjacencies
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Dependencies And Adjacencies

This page explains which crates `bijux-gnss-signal` may depend on directly and
which adjacent owners create more important review pressure than the cargo
graph alone would show.

## Direct Dependencies

- `bijux-gnss-core` for shared physical units, signal IDs, observation records,
  and sample container types
- `num-complex` for complex sample math
- `rustfft` for spectral analysis support
- `serde` and `schemars` for data-bearing contracts such as raw-IQ metadata and
  validation reports
- `thiserror` for signal-layer error reporting

## Adjacencies That Matter More Than The Cargo Graph

- `bijux-gnss-receiver` is the main runtime consumer whose orchestration needs
  can tempt stage policy into reusable signal code
- `bijux-gnss-nav` consumes signal semantics and validation helpers, but should
  not pull navigation judgment back into the signal layer
- `bijux-gnss-infra` handles capture metadata and persisted dataset state, but
  should not make repository file layout a signal concern
- `bijux-gnss-testkit` supplies deterministic signal truth and fixtures, but
  should remain a proof consumer rather than an API-design owner
- `bijux-gnss-policies` guards crate-shape and surface expectations, but does
  not own signal behavior itself

## Dependency Rules

- new dependencies are acceptable when they support reusable signal math,
  stable sample contracts, or durable serialization at the signal boundary
- new dependencies are suspect when they introduce filesystem I/O, operator
  policy, repository persistence, or receiver-runtime coupling
- higher-level crates may depend on `bijux-gnss-signal`; this crate must not
  depend on them just because a helper would be convenient

## Review Question

For every proposed dependency or export, ask whether it strengthens reusable
signal ownership or whether it is an attempt to sneak a higher-level concern
into the signal boundary.

## First Proof Check

Inspect `crates/bijux-gnss-signal/Cargo.toml`,
`crates/bijux-gnss-signal/docs/ARCHITECTURE.md`, and
`crates/bijux-gnss-signal/docs/BOUNDARY.md`. Then inspect
`crates/bijux-gnss-signal/src/api.rs`,
`crates/bijux-gnss-signal/src/obs_validation.rs`, and
`crates/bijux-gnss-signal/tests/integration_guardrails.rs` to confirm the
crate still depends downward while exporting a curated signal boundary upward.

## First Neighbor Proof Check

When a dependency question is really about runtime ownership, leave this page
for [05-bijux-gnss-receiver/foundation/dependencies-and-adjacencies](../../05-bijux-gnss-receiver/foundation/dependencies-and-adjacencies.md).
When the pressure is navigation-science coupling, inspect
[04-bijux-gnss-nav/foundation/dependencies-and-adjacencies](../../04-bijux-gnss-nav/foundation/dependencies-and-adjacencies.md).
