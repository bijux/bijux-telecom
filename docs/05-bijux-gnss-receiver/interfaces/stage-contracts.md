---
title: Stage Contracts
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Stage Contracts

Stage contracts define how the runtime exposes acquisition, tracking,
observation, and optional navigation behavior.

## Owned Stage Families

- acquisition engine and acquisition-assistance helpers in
  `src/pipeline/acquisition/`, `acquisition_assistance.rs`, and the related
  planning or refinement helpers
- tracking engine and channel-state or tracking-artifact types in
  `src/pipeline/tracking/`
- observation builders, residual reports, and measurement-quality reports in
  `src/pipeline/observations/`
- optional `Navigation` and `NavigationFilter` receiver-owned adapters over
  nav-owned science in `src/pipeline/navigation.rs` and
  `src/pipeline/navigation_filter.rs`
- `StepReport` and `StepStats` as handoff and report helpers

## Boundary Rule

These contracts are about runtime composition and handoff. The reusable signal
or navigation science used inside them still belongs to lower crates.

## Closest Proof

- `crates/bijux-gnss-receiver/src/pipeline/`
- `crates/bijux-gnss-receiver/tests/integration_acquisition_*.rs`
- `crates/bijux-gnss-receiver/tests/integration_tracking_*.rs`
- `crates/bijux-gnss-receiver/tests/integration_observations_*.rs`
- `crates/bijux-gnss-receiver/tests/integration_navigation_*.rs`
- `crates/bijux-gnss-receiver/docs/PIPELINE.md`

## Protecting Proof

Inspect the stage source family above together with `PIPELINE.md` and the
matching acquisition, tracking, observation, or navigation integration tests
before changing any stage contract. Those proofs show whether a change remains
stage-local or moves public runtime meaning.
