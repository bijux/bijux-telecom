---
title: Integration Seams
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Integration Seams

The crate stays reusable because its major regions connect through explicit
signal contracts instead of hidden runtime coupling.

## Main Seams

- catalog to codes:
  `catalog.rs` encodes canonical `SignalSpec` and component metadata that code
  and replica helpers can rely on
- codes to DSP:
  sampled and generated code behavior feeds local-code, replica, and spectrum
  helpers without assuming a receiver stage
- samples to DSP:
  `raw_iq.rs` and `samples.rs` define the input vocabulary that DSP helpers
  operate on
- validation to core observations:
  `obs_validation.rs` consumes `ObsEpoch` and related shared types from
  `bijux-gnss-core` while keeping signal-compatibility rules local
- api facade to private modules:
  `api.rs` turns the crate into a deliberate integration surface instead of
  forcing downstream crates to bind directly to internal layout

## Public Trait Seams

The lightweight traits `SignalSource`, `SampleSource`, `Correlator`, and
`SampleSink` are the narrow seams that let tests and higher-level crates
interoperate without transferring runtime ownership into this package.

## First Proof Check

Inspect `crates/bijux-gnss-signal/docs/TRAITS.md`,
`crates/bijux-gnss-signal/docs/PUBLIC_API.md`, and
`crates/bijux-gnss-signal/docs/ARCHITECTURE.md`. Then inspect
`crates/bijux-gnss-signal/src/api.rs`,
`crates/bijux-gnss-signal/src/raw_iq.rs`,
`crates/bijux-gnss-signal/src/samples.rs`, and
`crates/bijux-gnss-signal/src/obs_validation.rs` to confirm the seams remain
typed signal boundaries rather than hidden runtime adapters.

## First Neighbor Proof Check

When a seam starts carrying stage sequencing or channel orchestration, leave
this page for [05-bijux-gnss-receiver/architecture/integration-seams](../../05-bijux-gnss-receiver/architecture/integration-seams.md).
When it starts carrying estimator meaning, inspect
[04-bijux-gnss-nav/architecture/integration-seams](../../04-bijux-gnss-nav/architecture/integration-seams.md).
