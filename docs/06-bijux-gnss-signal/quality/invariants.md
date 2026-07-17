---
title: Invariants
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Invariants

These expectations should remain true as the crate grows.

## Boundary Invariants

- signal truth stays separate from receiver orchestration
- raw-IQ contract meaning stays separate from repository ingestion policy
- signal compatibility checks stay separate from navigation-quality judgment

## Behavioral Invariants

- supported code families have one canonical implementation
- reusable DSP helpers remain runtime-neutral
- long-duration sampling and phase helpers stay stable across chunk boundaries
- public exports stay grouped by signal meaning rather than convenience

## Review Invariant

If a change moves public signal behavior, the matching proof family and
handbook page should move with it.

## Protecting Proof

- `crates/bijux-gnss-signal/docs/TESTS.md`
- `crates/bijux-gnss-signal/src/api.rs`
- `crates/bijux-gnss-signal/tests/integration_guardrails.rs`
- `crates/bijux-gnss-signal/tests/integration_ca_code_reference.rs`
- `crates/bijux-gnss-signal/tests/integration_nco_long_duration_phase.rs`
- `crates/bijux-gnss-signal/tests/prop_obs_epoch_validation.rs`
