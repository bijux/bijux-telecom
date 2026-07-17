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

- format-family tests such as `integration_beidou_navigation_decode.rs`,
  `integration_glonass_navigation_decode.rs`, and
  `integration_rinex_observation_channels.rs`
- orbit and time reference tests such as
  `integration_broadcast_orbit_reference.rs`,
  `integration_sp3_reference_accuracy.rs`, and
  `integration_time_system_conversions.rs`
- correction-specific tests such as
  `integration_broadcast_ionosphere_residuals.rs`,
  `integration_iono_free_code.rs`, and
  `integration_measured_ionosphere.rs`
- estimator tests such as `integration_position.rs`,
  `integration_position_protection_levels.rs`,
  `integration_public_ppp_convergence.rs`, and
  `integration_rtk_ambiguity_fixing.rs`
- public-data and precise-product tests such as
  `integration_precise_products.rs`,
  `integration_bias_sinex_corrections.rs`, and
  `integration_public_spp_rtklib_position.rs`

## Why One Layer Is Never Enough

Passing a position test does not prove a decoder change is safe. Passing a
parser test does not prove PPP or RTK claims still hold. The crate needs proof
at the owning scientific layer and at the most relevant integration layer.

## Strong Example Families

- `integration_broadcast_orbit_reference.rs`
- `integration_sp3.rs` and `integration_sp3_products.rs`
- `integration_position*.rs`
- `integration_public_ppp_convergence.rs`
- `integration_rtk_*.rs`
- `integration_raim_*.rs`

## First Proof Check

Inspect `crates/bijux-gnss-nav/docs/TESTS.md`,
`crates/bijux-gnss-nav/docs/ESTIMATION.md`,
`crates/bijux-gnss-nav/docs/FORMATS.md`, and the example families above
together. That route shows whether proof is still distributed by scientific
owner instead of by convenience.
