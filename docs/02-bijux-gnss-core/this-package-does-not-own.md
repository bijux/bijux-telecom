---
title: This Package Does Not Own
audience: mixed
type: boundary
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# This Package Does Not Own

`bijux-gnss-core` is the shared meaning layer, not the shared behavior layer.
This ledger makes that distinction explicit so "used by many crates" does not
become a shortcut for moving local behavior into core.

## Explicit Refusals

- signal catalogs, code families, raw-IQ contracts, or DSP primitives
- orbit products, message parsers, correction models, or navigation estimators
- receiver runtime orchestration, ports, or staged execution policy
- dataset registry mechanics, run directories, sweep expansion, or provenance
  hashing
- command-line workflows, operator reports, or top-level user-facing policy

## Strongest Neighboring Owners

- [06-bijux-gnss-signal](../06-bijux-gnss-signal/) for signal-layer
  computation and sample-boundary behavior
- [04-bijux-gnss-nav](../04-bijux-gnss-nav/) for navigation science and
  estimator logic
- [05-bijux-gnss-receiver](../05-bijux-gnss-receiver/) for runtime execution
  and stage coordination
- [03-bijux-gnss-infra](../03-bijux-gnss-infra/) for repository persistence
  and dataset mechanics
- [01-bijux-gnss](../01-bijux-gnss/) for operator-facing command policy

## Review Trigger

If the proposed work sounds more like behavior than cross-package meaning, the
burden is on the change to prove that core is the right owner. Update this
page when recurring review arguments reveal a new boundary pressure worth
making durable. If the next proof surface is a stage scheduler, solver tuning
rule, filesystem policy, or command wording change, the owner is almost
certainly not `bijux-gnss-core`.

## First Neighbor Proof Check

- `../06-bijux-gnss-signal/index.md`
- `../04-bijux-gnss-nav/index.md`
- `../05-bijux-gnss-receiver/index.md`
- `../03-bijux-gnss-infra/index.md`
- `../01-bijux-gnss/index.md`
