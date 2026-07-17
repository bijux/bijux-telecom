---
title: Test Strategy
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# Test Strategy

`bijux-gnss-nav` needs several layers of proof because it owns both data
interpretation and estimator behavior.

## Main Proof Layers

- format-family tests for decoder correctness and rejection behavior
- orbit and time reference tests for state interpretation
- correction-specific tests for reusable GNSS-domain computations
- estimator tests for position, integrity, PPP, and RTK behavior
- public-data and precise-product tests for externally grounded evidence

## Why One Layer Is Never Enough

Passing a position test does not prove a decoder change is safe. Passing a
parser test does not prove PPP or RTK claims still hold. The crate needs proof
at the owning scientific layer and at the most relevant integration layer.

## Strong Example Families

- `integration_broadcast_orbit_reference`
- `integration_sp3` and `integration_sp3_products`
- `integration_position*`
- `integration_public_ppp_convergence`
- `integration_rtk_*`
