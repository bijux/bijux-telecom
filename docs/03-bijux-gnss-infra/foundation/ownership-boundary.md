---
title: Ownership Boundary
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Ownership Boundary

The cleanest way to keep `bijux-gnss-infra` useful is to name the stronger
adjacent owners explicitly.

## Boundary Ledger

| concern | owner | why infra should not absorb it |
| --- | --- | --- |
| command names, flags, top-level workflow policy | `bijux-gnss` | command UX is a user-facing boundary, not a repository-state contract |
| stage execution, ports, runtime metrics, receiver artifacts in memory | `bijux-gnss-receiver` | runtime orchestration needs state and policy that infra should consume, not define |
| signal codes, raw-IQ math, DSP primitives, sample conversions | `bijux-gnss-signal` | infra reads metadata and captures provenance, but does not own signal behavior |
| orbit products, corrections, estimators, navigation science | `bijux-gnss-nav` | reference comparison may pass through infra, but scientific meaning stays with nav |
| shared IDs, units, observation records, artifact envelope meaning | `bijux-gnss-core` | infra owns persisted footprint and repository interpretation, not cross-crate semantic language |

## Positive Rule

Infra should own the repository contract only when the main value is one shared
interpretation of datasets, runs, overrides, histories, or persisted artifacts.

## Negative Rule

If the proposed addition sounds more like product execution than repository
state, it likely belongs in a downstream product crate instead.

## First Proof Check

- `crates/bijux-gnss-infra/docs/BOUNDARY.md`
- adjacent root handbooks under `docs/01-*`, `docs/02-*`, `docs/04-*`,
  `docs/05-*`, and `docs/06-*`
