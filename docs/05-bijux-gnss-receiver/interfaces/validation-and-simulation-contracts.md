---
title: Validation And Simulation Contracts
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Validation And Simulation Contracts

These contracts cover the runtime-side proof surfaces published by the receiver
crate.

## Validation Contracts

- reference alignment and comparison helpers in `src/reference_validation.rs`
- runtime-side solution consistency reporting in `src/validation_report.rs`
- validation report builders, budgets, and science-policy records in
  `src/validation_report/` and `src/validation_helpers.rs`
- covariance realism summaries in `src/covariance_realism.rs` when navigation
  features are enabled

## Simulation Contracts

- synthetic receiver execution through the curated `src/sim/synthetic/`
  surface
- scenario-backed runtime validation and sensitivity helpers
- stage-accuracy, truth-table, and artifact-validation surfaces that exercise
  the receiver boundary directly

## Boundary Rule

These contracts prove receiver behavior. They should not become a repository
artifact framework or a replacement for lower-level truth ownership.

## Closest Proof

- `crates/bijux-gnss-receiver/src/reference_validation.rs`
- `crates/bijux-gnss-receiver/src/validation_report.rs`
- `crates/bijux-gnss-receiver/src/validation_report/tests/`
- `crates/bijux-gnss-receiver/src/sim/synthetic/`
- `crates/bijux-gnss-receiver/tests/integration_navigation_validation_run.rs`
- `crates/bijux-gnss-receiver/tests/integration_synthetic.rs`
- `crates/bijux-gnss-receiver/docs/REFERENCE_VALIDATION.md`
- `crates/bijux-gnss-receiver/docs/SIMULATION.md`
