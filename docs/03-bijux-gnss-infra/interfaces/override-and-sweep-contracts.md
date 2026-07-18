---
title: Override and Sweep Contracts
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-18
---

# Override and Sweep Contracts

Overrides and sweeps let repository workflows vary maintained receiver
profiles without turning configuration changes into unreviewed strings. Infra
owns the typed mutation mechanics; the receiver owns what each parameter means
at runtime.

## Experiment Expansion

```mermaid
flowchart TD
    profile["maintained receiver profile"]
    overrides["CommonOverrides"]
    sweep["SweepParameter list"]
    expansion["expand_sweep"]
    variants["typed run variants"]
    receiver["receiver config validation"]

    profile --> overrides --> variants
    profile --> sweep --> expansion --> variants --> receiver
```

## Contract Families

| family | owns | first proof |
| --- | --- | --- |
| common overrides | typed mutation of shared receiver-profile fields | `crates/bijux-gnss-infra/src/overrides/receiver_profile.rs` |
| direct override application | applying reviewed override values to a profile | `crates/bijux-gnss-infra/src/overrides/receiver_profile.rs` |
| sweep parsing | converting `PARAM=VALS` input into typed sweep parameters | `crates/bijux-gnss-infra/src/sweep.rs`, `crates/bijux-gnss-infra/src/overrides/sweep_parameters.rs` |
| sweep expansion | Cartesian expansion into reproducible run variants | `crates/bijux-gnss-infra/src/experiments.rs` |
| experiment specs | repository-facing batch-run descriptions | `crates/bijux-gnss-infra/src/experiments.rs` |

## Boundary Rules

- Infra owns typed variation, expansion, and reviewable experiment shape.
- Receiver owns parameter validation and runtime behavior after the profile is
  mutated.
- Command owns how operators request sweeps, not how sweep keys mutate a
  profile.
- Tests must prove expansion order, invalid keys, and value parsing because
  those details affect reproducibility.

## Reader Checks

- Can a reviewer see every run variant before execution?
- Does the same sweep input expand deterministically?
- Are invalid keys rejected at the infrastructure boundary?
- Does the documentation separate parameter mutation from receiver science?

## First Proof Check

Inspect `crates/bijux-gnss-infra/docs/OVERRIDES.md`,
`crates/bijux-gnss-infra/docs/EXPERIMENTS.md`,
`crates/bijux-gnss-infra/src/overrides/receiver_profile.rs`,
`crates/bijux-gnss-infra/src/overrides/sweep_parameters.rs`,
`crates/bijux-gnss-infra/src/experiments.rs`,
`crates/bijux-gnss-infra/src/sweep.rs`, and
`crates/bijux-gnss-infra/tests/integration_overrides.rs`.
