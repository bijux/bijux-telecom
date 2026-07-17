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

- reference alignment and comparison helpers
- runtime-side solution consistency reporting
- validation report builders, budgets, and science-policy records
- covariance realism summaries when navigation features are enabled

## Simulation Contracts

- synthetic receiver execution through the curated `sim` surface
- scenario-backed runtime validation and sensitivity helpers
- stage-accuracy and artifact-validation surfaces that exercise the receiver
  boundary directly

## Boundary Rule

These contracts prove receiver behavior. They should not become a repository
artifact framework or a replacement for lower-level truth ownership.

## Closest Proof

- `crates/bijux-gnss-receiver/src/reference_validation.rs`
- `crates/bijux-gnss-receiver/src/validation_report.rs`
- `crates/bijux-gnss-receiver/src/sim/synthetic/`
- `crates/bijux-gnss-receiver/docs/REFERENCE_VALIDATION.md`
- `crates/bijux-gnss-receiver/docs/SIMULATION.md`
