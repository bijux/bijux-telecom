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

- Galileo, BeiDou, and GLONASS time records
- civil-time parsing tied to navigation use
- conversion wrappers and time-offset evidence
- rollover interpretation and reference-day support

## Model Contracts

- atmosphere and ionosphere model seams
- antenna calibration and physical effect helpers
- celestial, ocean-tide, solid-earth-tide, and NeQuick support where public
  consumers need typed model results

## Boundary Rule

Foundational time meaning belongs in `core`. Navigation-specific interpretation
of external GNSS time systems and physically motivated model use belongs here.

## Closest Proof

- `crates/bijux-gnss-nav/src/time.rs`
- `crates/bijux-gnss-nav/src/models/`
- `crates/bijux-gnss-nav/docs/TIME.md`
- `crates/bijux-gnss-nav/docs/MODELS.md`
