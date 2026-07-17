---
title: Ownership Boundary
audience: mixed
type: explanation
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Ownership Boundary

The safest way to keep `bijux-gnss-core` healthy is to be explicit about the
adjacent owners it should not replace.

## Boundary Ledger

| concern | owner | why core should not absorb it |
| --- | --- | --- |
| signal catalogs, code families, sample contracts, DSP primitives | `bijux-gnss-signal` | these are reusable signal behaviors, not generic contract records |
| orbit products, corrections, estimators, solver-side models | `bijux-gnss-nav` | these are scientific behaviors over records, not the records themselves |
| staged runtime execution, ports, tracking sessions, receiver artifacts in memory | `bijux-gnss-receiver` | runtime orchestration needs state and policy that core intentionally avoids |
| datasets, run layout, persisted manifests, experiment sweeps, provenance hashing | `bijux-gnss-infra` | repository persistence and operational history are stronger owners than envelope contracts |
| operator commands and top-level workflow composition | `bijux-gnss` | command vocabulary is a user-facing boundary, not a foundational record layer |

## Positive Rule

Core should own the record or invariant only when downstream crates need to
agree on the meaning even if they disagree on implementation strategy.

## Negative Rule

If the proposed addition makes more sense as behavior than as shared meaning,
it likely belongs elsewhere.

## Typical Boundary Failures

- a downstream crate exports an implementation detail, then core absorbs it
  because multiple crates started touching it
- an artifact file layout concern gets confused with the artifact payload
  contract itself
- a solver-specific record is generalized too early and weakens the shared
  language for everyone

## First Proof Check

- `crates/bijux-gnss-core/docs/CONTRACT_MAP.md`
- `crates/bijux-gnss-core/src/api.rs`
- adjacent root handbooks under `docs/03-*` through `docs/06-*`
