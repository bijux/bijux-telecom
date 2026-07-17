---
title: Error Model
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# Error Model

Navigation code has more than one honest failure mode. The architecture should
keep them distinct.

## Main Failure Families

- parse rejection when an external message or product cannot be interpreted
- scientific refusal when a solver should not claim a solution
- integrity evidence that reports suspicion without pretending the data is
  clean
- unsupported-product or unsupported-signal outcomes when the crate knows the
  contract but the input does not satisfy it

## Why Distinction Matters

A parser rejection is not the same as an underdetermined RAIM exclusion. A PPP
product-policy downgrade is not the same as a malformed SP3 line. If those
cases collapse into generic failure handling, downstream trust gets weaker.

## Closest Proof

- `crates/bijux-gnss-nav/src/formats/`
- `crates/bijux-gnss-nav/src/estimation/position/solver/solution_outcome.rs`
- `crates/bijux-gnss-nav/src/estimation/solution_claims.rs`
