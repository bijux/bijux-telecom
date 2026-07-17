---
title: Dependency Direction
audience: mixed
type: explanation
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Dependency Direction

`bijux-gnss-core` should be depended on far more often than it depends outward.

## Direction Rule

- `bijux-gnss-signal`, `bijux-gnss-nav`, `bijux-gnss-receiver`,
  `bijux-gnss-infra`, and `bijux-gnss` may depend on core
- core must not depend on those higher-level crates

## Why The Rule Exists

The crate’s value is portability of meaning. Once it starts importing runtime,
solver, or repository behavior, its records stop being safe common language and
start carrying one owner’s assumptions.

## Healthy Pressure

- higher-level crates pull types from core
- core defines no knowledge of their scheduling, persistence, or command paths

## Unhealthy Pressure

- a convenience helper in `receiver` or `infra` gets pulled into core to avoid
  duplication
- artifact-envelope logic starts needing filesystem layout context
- navigation-solver details become encoded in supposedly shared record fields

## First Proof Check

- `crates/bijux-gnss-core/Cargo.toml`
- `crates/bijux-gnss-core/tests/integration_guardrails.rs`
