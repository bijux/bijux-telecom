---
title: Execution Model
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Execution Model

`bijux-gnss-signal` is computational, not orchestrational. It transforms
in-memory values and returns results to callers that own the surrounding
workflow.

## Main Execution Shape

- catalog helpers resolve signal meaning and physical conversions
- code-family functions generate or sample deterministic code behavior
- DSP helpers transform samples, phases, spectra, and tracking math without
  deciding when a receiver stage runs
- validation helpers inspect observation epochs and report signal compatibility

## Runtime-Neutral Rule

The crate may hold local state when the math requires it, such as NCO state or
local-code model state. That does not make it a runtime owner. The state exists
to model signal behavior, not to schedule channels, control a pipeline, or
manage artifacts.

## Why This Matters

The same helper should be usable from:

- receiver runtime
- command-boundary validation flows
- synthetic generation support
- standalone tests and reference checks

If a helper can work only inside one receiver execution context, it probably
belongs in `bijux-gnss-receiver`, not here.

## First Proof Check

- `crates/bijux-gnss-signal/src/api.rs`
- `crates/bijux-gnss-signal/src/dsp/local_code.rs`
- `crates/bijux-gnss-signal/src/dsp/nco.rs`
- `crates/bijux-gnss-signal/src/dsp/replica.rs`
- `crates/bijux-gnss-signal/src/obs_validation.rs`
