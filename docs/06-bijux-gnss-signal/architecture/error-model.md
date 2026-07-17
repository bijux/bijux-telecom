---
title: Error Model
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Error Model

`SignalError` is the crate's shared error vocabulary for invalid signal inputs,
unsupported definitions, and malformed DSP requests.

## Error Families

- invalid physical input:
  sample rates, code rates, carrier frequencies, code phase, and elapsed
  duration
- unsupported signal meaning:
  unsupported PRNs, missing GLONASS frequency channels, or incomplete signal
  definitions
- malformed sequences:
  empty code streams, empty navigation symbols, or correlation length mismatch
- invalid analysis configuration:
  front-end filter and spectrum-analysis failures

## Architectural Role

The error model protects reusable signal boundaries by failing early on bad
signal assumptions. It should stay focused on signal correctness. Repository
I/O failures, runtime orchestration failures, and operator policy failures
belong elsewhere.

## First Proof Check

Inspect `crates/bijux-gnss-signal/src/error.rs`,
`crates/bijux-gnss-signal/docs/BOUNDARY.md`, and
`crates/bijux-gnss-signal/docs/CONTRACTS.md`. Then inspect
`crates/bijux-gnss-signal/tests/integration_raw_iq_metadata.rs`,
`crates/bijux-gnss-signal/tests/integration_signal_spectrum_front_end_low_pass.rs`,
and `crates/bijux-gnss-signal/tests/prop_obs_epoch_validation.rs` to confirm
that failures still map to signal-domain contracts rather than higher-level
workflow errors.
