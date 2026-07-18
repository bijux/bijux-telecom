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
  the signal catalog encodes canonical `SignalSpec` and component metadata
  that code and replica helpers can rely on
- codes to DSP:
  sampled and generated code behavior feeds local-code, replica, and spectrum
  helpers without assuming a receiver stage
- samples to DSP:
  raw-IQ and sample models define the input vocabulary that DSP helpers operate
  on
- validation to core observations:
  observation validation consumes `ObsEpoch` and related shared types from core
  while keeping signal-compatibility rules local
- api facade to private modules:
  the public API facade turns the crate into a deliberate integration surface
  instead of forcing downstream crates to bind directly to internal layout

## Public Trait Seams

The lightweight traits `SignalSource`, `SampleSource`, `Correlator`, and
`SampleSink` are the narrow seams that let tests and higher-level crates
interoperate without transferring runtime ownership into this package.

## First Proof Check

Start with the signal [trait guide](../../../crates/bijux-gnss-signal/docs/TRAITS.md),
[public API guide](../../../crates/bijux-gnss-signal/docs/PUBLIC_API.md), and
[architecture guide](../../../crates/bijux-gnss-signal/docs/ARCHITECTURE.md).
Then confirm the code seams through the [public API facade](../../../crates/bijux-gnss-signal/src/api.rs),
[raw-IQ model](../../../crates/bijux-gnss-signal/src/raw_iq.rs),
[sample model](../../../crates/bijux-gnss-signal/src/samples.rs), and
[observation validation source](../../../crates/bijux-gnss-signal/src/obs_validation.rs).
Those surfaces should remain typed signal boundaries, not hidden runtime
adapters.

## First Neighbor Proof Check

When a seam starts carrying stage sequencing or channel orchestration, leave
this page for the [receiver integration-seams guide](../../bijux-gnss-receiver/architecture/integration-seams.md).
When it starts carrying estimator meaning, inspect
the [navigation integration-seams guide](../../bijux-gnss-nav/architecture/integration-seams.md).
