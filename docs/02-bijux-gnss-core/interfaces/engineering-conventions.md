---
title: Engineering Conventions
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Engineering Conventions

This page absorbs the old root-level conventions guide into the crate that owns
shared units, identifiers, and cross-crate physical meaning.

## Shared Convention Families

- Doppler sign conventions
- carrier-phase direction and units
- shared reference-frame vocabulary such as WGS-84 ECEF and ENU
- the expectation that record meanings stay explicit instead of relying on
  caller folklore

## Boundary Rule

Conventions that multiple higher crates must read the same way belong here.
Implementation tactics for one solver, one signal family, or one runtime stage
still belong in their local owners.

## Protecting Proof

- `crates/bijux-gnss-core/docs/CONTRACTS.md`
- `crates/bijux-gnss-core/docs/INVARIANTS.md`
- `crates/bijux-gnss-core/src/observation/`
