---
title: Integration Seams
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# Integration Seams

This page names the places where `bijux-gnss-nav` intentionally meets
neighboring crates without surrendering ownership.

## Public API Seam

`src/api.rs` is the deliberate integration seam for downstream crates. If a
consumer needs an internal module path instead of `api.rs`, the crate boundary
may be too leaky.

## Product Provider Seams

- `EphemerisProvider` in `src/orbits/ephemeris.rs`
- `ProductsProvider` in `src/formats/precise_products/mod.rs`
- bias-provider traits in `src/corrections/biases.rs`

These seams let solvers and correction families consume scientific inputs
without hardwiring every caller to one storage shape.

## Estimator Integration Seams

- `MeasurementModel` and related EKF traits in `src/estimation/ekf/traits.rs`
- `NavigationEngine`, `PositionRuntime`, and solver surfaces consumed by
  runtime layers
- PPP and RTK product-policy surfaces that let higher layers decide which
  precise inputs are available without redefining the scientific contract

## Boundary Rule

Integration seams should pass scientific inputs and outputs. They should not
smuggle repository paths, CLI policy, or receiver scheduling state into the
crate.
