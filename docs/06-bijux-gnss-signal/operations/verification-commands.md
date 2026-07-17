---
title: Verification Commands
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Verification Commands

Run the smallest honest proof set that matches the changed owner.

## Common Commands

```sh
cargo test -p bijux-gnss-signal --test integration_signal_component_registry
cargo test -p bijux-gnss-signal --test integration_signal_spectrum_cboc
cargo test -p bijux-gnss-signal --test integration_raw_iq_metadata
cargo test -p bijux-gnss-signal --test prop_obs_epoch_validation
cargo test -p bijux-gnss-signal
```

## Matching Command To Change

- code-family changes:
  run the relevant reference and continuity tests for that family, such as
  `integration_ca_code_reference`,
  `integration_gps_l2c_multiplex`,
  `integration_galileo_e5_reference`, or
  `integration_glonass_l1_reference`
- DSP changes:
  run the relevant long-duration, spectrum, or loop-behavior tests such as
  `integration_nco_long_duration_phase`,
  `integration_replica_continuity`, or
  `integration_signal_spectrum_front_end_low_pass`
- metadata or sample changes:
  run `integration_raw_iq_metadata` and
  `integration_iq_sample_conversion`
- validation changes:
  run `prop_obs_epoch_validation` and the surrounding signal-family tests that
  consume those contracts

Treat `crates/bijux-gnss-signal/docs/TESTS.md` as the proof map when a change
crosses code families, DSP, and sample contracts. Run the full crate test
command when multiple proof families moved together.

## Bad Verification Pattern

Do not rely on one unrelated green test just because it exercises the crate.
This package has many proof families, and the changed owner should be obvious
from the verification set.
