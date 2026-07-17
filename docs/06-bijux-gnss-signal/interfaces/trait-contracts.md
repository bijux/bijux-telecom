---
title: Trait Contracts
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Trait Contracts

The public traits are small on purpose. They exist to provide signal-adjacent
integration seams without importing receiver ownership.

## Public Traits

- `SignalSource`
- `SampleSource`
- `Correlator`
- `SampleSink`

## Contract Shape

- `SignalSource` and `SampleSource` abstract frame delivery
- `Correlator` abstracts prompt/early/late correlation without dictating a
  runtime engine
- `SampleSink` abstracts destination writing for sample frames

## Boundary Rule

These traits should remain thin. If a proposed method needs scheduling policy,
artifact metadata, or run-level lifecycle control, the trait has outgrown this
crate.
