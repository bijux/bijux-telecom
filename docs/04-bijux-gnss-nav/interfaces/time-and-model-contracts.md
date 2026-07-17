---
title: Time And Model Contracts
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# Time And Model Contracts

These contracts cover the scientific support surfaces that are public because
multiple navigation families need them.

## Time Contracts

- Galileo, BeiDou, and GLONASS time records in `src/time.rs`
- civil-time parsing tied to navigation use
- conversion wrappers and time-offset evidence
- rollover interpretation and reference-day support in `src/time/rollover.rs`

## Model Contracts

- atmosphere and ionosphere model seams in `src/models/atmosphere.rs` and the
  NeQuick family
- antenna calibration and physical effect helpers in `src/models/antenna.rs`
- celestial, ocean-tide, solid-earth-tide, and NeQuick support where public
  consumers need typed model results

## Boundary Rule

Foundational time meaning belongs in `core`. Navigation-specific interpretation
of external GNSS time systems and physically motivated model use belongs here.

## Closest Proof

- `crates/bijux-gnss-nav/src/time.rs`
- `crates/bijux-gnss-nav/src/time/rollover.rs`
- `crates/bijux-gnss-nav/src/models/`
- `crates/bijux-gnss-nav/tests/integration_utc_leap_seconds.rs`
- `crates/bijux-gnss-nav/tests/integration_troposphere_elevation.rs`
- `crates/bijux-gnss-nav/docs/TIME.md`
- `crates/bijux-gnss-nav/docs/MODELS.md`

## Protecting Proof

Inspect the time and model source families above together with
`crates/bijux-gnss-nav/tests/integration_time_system_conversions.rs` and
`crates/bijux-gnss-nav/tests/integration_troposphere_elevation.rs` to confirm
navigation-specific time interpretation and physical models still match checked
scientific proof.
