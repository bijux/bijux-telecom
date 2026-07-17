---
title: State And Persistence
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# State And Persistence

This crate allows local computational state and forbids repository persistence.

## Allowed State

- `NcoState` and `Nco` hold oscillator progress
- `LocalCodeModel` and `ReplicaCodeModel` hold reusable sampling context
- tracking inputs, updates, and uncertainty structures carry short-lived DSP
  state across calls

## Forbidden Persistence

- no filesystem-backed sample capture policy
- no run directory or artifact layout
- no repository metadata ledger
- no long-lived runtime session ownership

## Contract-Bearing Data

`RawIqMetadata`, validation reports, and sample buffers are durable data
contracts, but they are not persistence policy. This crate defines their
meaning; it does not decide where or how another crate stores them.

## First Proof Check

Inspect `crates/bijux-gnss-signal/docs/RAW_IQ.md`,
`crates/bijux-gnss-signal/docs/SAMPLES.md`,
`crates/bijux-gnss-signal/docs/VALIDATION.md`, and
`crates/bijux-gnss-signal/docs/BOUNDARY.md`. Then inspect
`crates/bijux-gnss-signal/src/raw_iq.rs`,
`crates/bijux-gnss-signal/src/samples.rs`,
`crates/bijux-gnss-signal/src/dsp/nco.rs`, and
`crates/bijux-gnss-signal/tests/integration_raw_iq_metadata.rs` to confirm
the crate still owns typed in-memory state without crossing into repository
storage policy.
