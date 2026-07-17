---
title: Known Limitations
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# Known Limitations

This handbook does not pretend the crate is small or simple.

## Main Limitations

- the public API is necessarily broad because several downstream crates consume
  scientific behavior directly
- the test surface is expensive enough that choosing the right validation slice
  requires judgment
- constellation-specific, precise-product, and estimator families can interact
  in ways that make local changes feel broader than they first appear
- local matrix and model support are intentionally crate-owned, which requires
  periodic review to ensure they do not become generic utility sprawl

## First Proof Check

- `crates/bijux-gnss-nav/docs/TESTS.md`
- `crates/bijux-gnss-nav/tests/long_run_stability.rs`
- `crates/bijux-gnss-nav/tests/integration_public_ppp_convergence.rs`
