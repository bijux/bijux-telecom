---
title: Extensibility Model
audience: mixed
type: explanation
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Extensibility Model

`bijux-gnss-infra` should extend by deepening named repository contracts rather
than by adding broad utility surfaces.

## Preferred Extension Paths

- extend `datasets/` when repository-side metadata interpretation grows
- extend `run_layout/` when persisted footprint rules become richer
- extend `overrides/` and `sweep.rs` when reproducible variation gains typed
  cases
- extend `artifact_inspection/` or `validate_reference.rs` when persisted
  evidence workflows need clearer repository ownership

## Extension Smells

- new helper with no durable dataset, layout, validation, or provenance owner
- wrapping a lower-level API unchanged and calling that infrastructure
- storing command-specific policy as a general repository contract

## First Proof Check

- `crates/bijux-gnss-infra/docs/CONTRACTS.md`
- `crates/bijux-gnss-infra/tests/integration_guardrails.rs`
