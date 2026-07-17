---
title: Dependency Direction
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Dependency Direction

`bijux-gnss-infra` is allowed to depend on lower-level product crates, but only
to define repository-facing contracts around them.

## Direction Rule

- infra may depend on `core`, `signal`, `nav`, and `receiver`
- higher-level callers such as `bijux-gnss` may depend on infra
- infra should not become the place where unrelated helper code lands merely
  because it already depends on many things

## Why The Rule Exists

This crate sits at a legitimate aggregation boundary. That makes it useful and
dangerous at the same time. The dependency rule is what keeps the aggregation
honest.

## Healthy Pressure

- wrapping lower-level contracts in repository state and persisted-footprint
  rules
- exposing typed helpers that prevent each caller from rebuilding path or
  manifest logic

## Unhealthy Pressure

- importing product behavior without adding repository ownership
- centralizing convenience helpers that have no dataset, layout, validation, or
  provenance contract
- letting command semantics or runtime policy bleed inward because infra is
  already "nearby"

## First Proof Check

- `crates/bijux-gnss-infra/Cargo.toml`
- `crates/bijux-gnss-infra/src/api.rs`
- `crates/bijux-gnss-infra/docs/BOUNDARY.md`
- `crates/bijux-gnss-infra/tests/integration_guardrails.rs`
