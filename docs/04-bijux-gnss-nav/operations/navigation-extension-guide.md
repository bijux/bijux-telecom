---
title: Navigation Extension Guide
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# Navigation Extension Guide

This page replaces the old root-level navigation-extension note. Extending
navigation behavior belongs with the crate that owns orbit state, correction
law, and estimator meaning.

## Add A Correction Model

1. implement the model in
   `crates/bijux-gnss-nav/src/corrections/`
2. wire it through the owning correction flow instead of hiding it inside a
   receiver or command helper
3. add proof in `crates/bijux-gnss-nav/tests/` that covers both nominal use and
   the failure or refusal mode that protects callers
4. update public navigation contracts when the result shape or solver promise
   changes

Common existing owners worth matching before adding a new file are
`broadcast_ionosphere_residuals.rs`, `dual_frequency.rs`,
`measured_ionosphere.rs`, `phase_windup.rs`, and the combination helpers such
as `iono_free_code.rs` and `narrow_lane.rs`.

## Extend Satellite Clock Handling

1. keep broadcast clock behavior with the owning clock model rather than
   scattering bias terms across unrelated solvers
2. route external clock sources through the public product boundary that
   already owns precise clock interpretation
3. preserve the rule that precise-clock bias replaces the relevant broadcast
   bias contribution rather than stacking on top of it blindly
4. prove both broadcast fallback and precise-clock override behavior with
   targeted tests such as `integration_clk_reference_accuracy.rs` or
   `integration_broadcast_clock_reference.rs`

## Extend An Estimator Surface

1. place new position behavior under `src/estimation/position/`, PPP behavior
   under `src/estimation/ppp/`, and RTK behavior under `src/estimation/rtk/`
2. keep shared filter primitives with `src/estimation/ekf/` only when they are
   genuinely reusable across solver families
3. add a refusal, downgrade, or integrity-focused proof when the extension can
   fail in a scientifically meaningful way
4. update public estimation contracts when downstream crates are meant to rely
   on the new behavior

## Boundary Rule

If the new behavior is reusable signal math, move it into
`bijux-gnss-signal`. If the new behavior is runtime scheduling or stage
plumbing, keep it in `bijux-gnss-receiver`. `bijux-gnss-nav` owns the science,
not every caller convenience.
