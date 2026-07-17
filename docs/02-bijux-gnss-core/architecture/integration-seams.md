---
title: Integration Seams
audience: mixed
type: explanation
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Integration Seams

The crate’s seams are not service boundaries. They are contract boundaries.

## Main Seams

- `api.rs` is the seam between internal module layout and downstream imports
- artifact payload validation is the seam between record shape and persisted
  trust
- observation records are the seam between signal processing and navigation
  interpretation
- navigation-solution records are the seam between solver internals and shared
  downstream result meaning
- diagnostic records are the seam between failures and reviewable structured
  evidence

## Why These Seams Matter

Weak seams here create false convenience in higher-level crates. A record can
look stable while still leaking implementation assumptions if the seam is not
named clearly.

## First Proof Check

- `crates/bijux-gnss-core/src/api.rs`
- `crates/bijux-gnss-core/src/artifact/`
- `crates/bijux-gnss-core/src/observation/`
- `crates/bijux-gnss-core/src/nav_solution.rs`
