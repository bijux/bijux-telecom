---
title: Invariants
audience: mixed
type: explanation
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Invariants

The most important `bijux-gnss-core` invariants are the ones downstream crates
quietly depend on every day.

## Public-Surface Invariants

- public structs and free functions remain deliberately re-exported through
  `src/api.rs`
- callers should not need private module paths to use stable core meaning

## Artifact Invariants

- artifact payload validators enforce semantic coherence, not just parse shape
- non-finite or internally inconsistent persisted values are rejected rather
  than normalized silently

## Time Invariants

- GPS, UTC, TAI, and sample-trace conversions remain deterministic under the
  protecting property tests and regression corpus

## Boundary Invariants

- `bijux-gnss-core` stays free of higher-level workspace crate dependencies
- the crate remains a contract foundation instead of drifting into runtime or
  repository policy

## Protecting Proof

- `crates/bijux-gnss-core/docs/INVARIANTS.md`
- `crates/bijux-gnss-core/tests/`
