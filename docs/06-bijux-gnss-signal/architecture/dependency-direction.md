---
title: Dependency Direction
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Dependency Direction

The architectural rule is simple: shared meaning enters from below, reusable
signal logic is assembled here, and higher-level policy stays above this crate.

## Inbound Direction

- `bijux-gnss-core` supplies shared types and physical units
- numeric crates support implementation detail, not product ownership
- `serde`, `schemars`, and `thiserror` support durable contracts and errors

## Internal Direction

- `api.rs` depends on private owners and re-exports them deliberately
- `catalog.rs` depends on code-family constants and shared core types
- `obs_validation.rs`, `raw_iq.rs`, and `samples.rs` depend on shared types and
  local contracts, not on receiver runtime
- DSP modules may depend on catalog or code definitions where signal meaning is
  required, but they should not depend on higher-level execution logic

## Outbound Direction

- `bijux-gnss-receiver`, `bijux-gnss`, and other higher crates consume this
  crate
- this crate must not depend on command, receiver, infrastructure, or
  navigation packages

## Review Rule

Any change that would reverse the dependency direction by pulling a higher
layer into `bijux-gnss-signal` is an architectural regression, even if the
helper itself appears useful.
