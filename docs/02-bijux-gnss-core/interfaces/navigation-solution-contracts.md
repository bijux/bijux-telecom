---
title: Navigation Solution Contracts
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Navigation Solution Contracts

Navigation-solution contracts describe solver output without requiring
downstream crates to know how the solver internally reached it.

## Main Record Families

- `NavSolutionEpoch`
- `NavResidual`
- `NavConstellationResidualRms`
- `InterSystemBias`
- validity, refusal, lifecycle-state, provenance, and uncertainty classes

## Boundary Rule

The records should be rich enough for downstream inspection and artifact
storage, but they should not encode one estimator’s internal state machine as
the shared contract language for everyone else.

## Protecting Proof

- `crates/bijux-gnss-core/src/nav_solution.rs`
- `crates/bijux-gnss-core/src/observation/navigation.rs`
- `crates/bijux-gnss-core/docs/CONTRACTS.md`
- `crates/bijux-gnss-core/docs/INVARIANTS.md`
