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

## First Proof Check

Inspect `crates/bijux-gnss-signal/docs/TESTS.md`,
`crates/bijux-gnss-signal/tests/data/`,
`crates/bijux-gnss-signal/tests/support/`, and the family-specific reference
tests such as `integration_ca_code_reference.rs` and
`integration_galileo_e5_reference.rs`. Those surfaces show whether a fixture
change is defending canonical truth or merely teaching the tests to accept
drift.
