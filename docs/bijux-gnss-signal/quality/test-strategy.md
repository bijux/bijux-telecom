---
title: Test Strategy
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Test Strategy

`bijux-gnss-signal` needs layered proof because it publishes canonical signal
behavior that many higher-level crates reuse.

## Main Proof Layers

- guardrail tests such as `integration_guardrails.rs` that keep public surface
  and crate-shape pressure honest
- registry and reference tests such as
  `integration_gps_l5_registry.rs`,
  `integration_galileo_e5_reference.rs`, and
  `integration_glonass_l1_reference.rs`
- long-duration continuity tests such as
  `integration_ca_code_long_duration_phase.rs`,
  `integration_carrier_wipeoff_long_duration.rs`,
  `integration_nco_long_duration_phase.rs`, and
  `integration_replica_continuity.rs`
- spectrum and front-end tests such as
  `integration_signal_spectrum_bpsk_low_rate.rs`,
  `integration_signal_spectrum_cboc.rs`, and
  `integration_signal_spectrum_front_end_low_pass.rs`
- raw-IQ metadata and sample-conversion tests in
  `integration_raw_iq_metadata.rs` and
  `integration_iq_sample_conversion.rs`
- property tests in `prop_ca_code.rs`, `prop_nco.rs`, and
  `prop_obs_epoch_validation.rs`

## Why Layers Matter

Reference tests prove canonical signal truth. Continuity tests prove chunk and
time stability. Property tests prove structural rules. One layer cannot replace
the others without creating blind spots.

## Protecting Proof

- the [signal test guide](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-signal/docs/TESTS.md)
- the [guardrail test](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-signal/tests/integration_guardrails.rs)
- the [component registry test](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-signal/tests/integration_signal_component_registry.rs)
- the [GPS C/A reference test](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-signal/tests/integration_ca_code_reference.rs)
- the [NCO long-duration phase test](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-signal/tests/integration_nco_long_duration_phase.rs)
- the [raw-IQ metadata test](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-signal/tests/integration_raw_iq_metadata.rs)
- the [observation validation property test](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-signal/tests/prop_obs_epoch_validation.rs)
