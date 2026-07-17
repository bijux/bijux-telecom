---
title: DSP Contracts
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# DSP Contracts

The DSP surface publishes reusable signal math, not receiver-stage behavior.

## Main Public DSP Contracts

- `FrontEndFilterSpec`, `FrontEndFirFilter`, and response-measurement helpers
- `LocalCodeModel` and local-code construction helpers
- `Nco` and `NcoState`
- replica and wipeoff helpers such as `sample_modulated_replica_at_time` and
  `wipeoff_carrier`
- spectrum summary and null-detection helpers
- tracking-loop inputs, updates, thresholds, and discriminators

## What Callers May Rely On

- these helpers are reusable without a specific runtime container
- state-bearing DSP types model signal behavior, not workflow ownership
- front-end, spectrum, and tracking helpers are part of the public signal math
  boundary

## What Callers Must Not Assume

- that these helpers define receiver scheduling or artifact policy
- that every internal numeric helper in `dsp/math.rs` is a public promise

## Protecting Proof

- `crates/bijux-gnss-signal/docs/DSP.md`
- `crates/bijux-gnss-signal/src/dsp/front_end.rs`
- `crates/bijux-gnss-signal/src/dsp/nco.rs`
- `crates/bijux-gnss-signal/src/dsp/replica.rs`
- `crates/bijux-gnss-signal/src/dsp/spectrum.rs`
- `crates/bijux-gnss-signal/src/dsp/tracking.rs`
- `crates/bijux-gnss-signal/tests/integration_nco_long_duration_phase.rs`
- `crates/bijux-gnss-signal/tests/integration_replica_continuity.rs`
- `crates/bijux-gnss-signal/tests/integration_signal_spectrum_cboc.rs`
