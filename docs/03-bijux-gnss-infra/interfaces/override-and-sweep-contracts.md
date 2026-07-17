---
title: Override and Sweep Contracts
audience: mixed
type: explanation
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Override and Sweep Contracts

Override and sweep contracts let callers vary maintained receiver profiles in a
typed way.

## Main Owned Records And Helpers

- `CommonOverrides`
- `apply_common_overrides`
- `apply_overrides`
- `apply_sweep_value`
- `ExperimentSpec`
- `SweepParameter`
- `parse_sweep`
- `expand_sweep`

## Why They Matter

These surfaces keep experiment variation reviewable. Without them, repository
batch work collapses back into ad hoc string handling and per-command mutation.

## Boundary Rule

Infra owns the typed mutation and expansion mechanics. It does not own the
scientific meaning of the underlying receiver parameters.

## Protecting Proof

- `crates/bijux-gnss-infra/docs/OVERRIDES.md`
- `crates/bijux-gnss-infra/docs/EXPERIMENTS.md`
- `crates/bijux-gnss-infra/tests/integration_overrides.rs`
