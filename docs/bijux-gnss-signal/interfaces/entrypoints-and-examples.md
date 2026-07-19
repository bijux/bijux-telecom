---
title: Entrypoints And Examples
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Entrypoints And Examples

Use these entrypoints when choosing a public signal surface.

## Common Starting Points

- open `bijux_gnss_signal::api::signal_registry` when the caller needs
  supported-signal lookup
- open a family-specific generator such as `generate_ca_code` or
  `generate_galileo_e1b_code` when the caller needs canonical code behavior
- open `default_local_code_model_for_signal` or
  `sample_modulated_replica_at_time` when the caller needs reusable DSP
  building blocks
- open `RawIqMetadata` and `iq_i16_to_samples` when the caller is bridging a
  capture contract into normalized samples
- open `validate_obs_epochs` or `check_dual_frequency_observations` when the
  caller needs signal-compatibility proof

## Example Navigation Path

If a caller begins with raw capture bytes and wants signal-layer validation,
the durable path is:

1. describe the capture with `RawIqMetadata`
2. normalize samples through the published conversion helpers
3. use catalog, code, or DSP helpers as needed
4. apply validation helpers to shared observation records once they exist

## Protecting Proof

Start with the signal [public API guide](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-signal/docs/PUBLIC_API.md),
[raw-IQ guide](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-signal/docs/RAW_IQ.md),
[validation guide](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-signal/docs/VALIDATION.md), and
[public API facade](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-signal/src/api.rs). Then confirm
the reader path through the [sample-conversion test](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-signal/tests/integration_iq_sample_conversion.rs),
[raw-IQ metadata test](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-signal/tests/integration_raw_iq_metadata.rs),
and [observation validation property test](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-signal/tests/prop_obs_epoch_validation.rs).
