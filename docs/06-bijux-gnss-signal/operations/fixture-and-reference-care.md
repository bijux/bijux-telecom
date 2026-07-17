---
title: Fixture And Reference Care
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Fixture And Reference Care

The support and reference-data tree is part of the contract, not disposable
test clutter.

## Owned Proof Assets

- `tests/data/` contains checked-in reference catalogs
- `tests/support/` contains independently generated helpers and validation
  support
- family-specific reference tests confirm canonical code and signal behavior

## Care Rules

- change reference catalogs only when the underlying canonical behavior truly
  changed
- keep support helpers tied to one durable proof purpose
- explain why a reference update is scientifically credible, not just why it
  makes tests pass

## Boundary Rule

Reference generators may live in test support, but production truth still
belongs in the crate modules. Test data should verify canonical behavior, not
replace it.
