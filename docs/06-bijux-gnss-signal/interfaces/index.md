---
title: Interfaces
audience: mixed
type: index
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Interfaces

Open this section when the question is what `bijux-gnss-signal` publicly
promises to downstream crates.

## Read These First

- open [API Surface](api-surface.md) first when the question is what this
  crate exports at all
- open [Trait Contracts](trait-contracts.md) when the question is about the
  public integration seams
- open [Raw IQ And Sample Contracts](raw-iq-and-sample-contracts.md) when the
  change touches metadata or normalized samples

## Pages In This Section

- [API Surface](api-surface.md)
- [Public Imports](public-imports.md)
- [Code Contracts](code-contracts.md)
- [DSP Contracts](dsp-contracts.md)
- [Raw IQ And Sample Contracts](raw-iq-and-sample-contracts.md)
- [Signal Model Assumptions](signal-model-assumptions.md)
- [Validation Contracts](validation-contracts.md)
- [Trait Contracts](trait-contracts.md)
- [Entrypoints And Examples](entrypoints-and-examples.md)
- [Compatibility Commitments](compatibility-commitments.md)

## First Public Surfaces

- `crates/bijux-gnss-signal/src/api.rs`
- `crates/bijux-gnss-signal/src/raw_iq.rs`
- `crates/bijux-gnss-signal/src/samples.rs`
- `crates/bijux-gnss-signal/src/obs_validation.rs`

## Leave This Section When

- leave for [Architecture](../architecture/) when the question is about module
  placement rather than public contract
- leave for [Operations](../operations/) when the interface is clear and the
  next question is how to change it safely
