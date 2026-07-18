---
title: Architecture
audience: mixed
type: index
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Architecture

Open this section when the question is where signal behavior lives in code and
why the source tree is partitioned the way it is.

## Structural Shape

```mermaid
flowchart LR
    api["api.rs<br/>curated public surface"]
    catalog["catalog.rs"]
    codes["codes/"]
    dsp["dsp/"]
    samples["raw_iq.rs and samples.rs"]
    validation["obs_validation.rs"]
    downstream["receiver nav cli infra tests"]

    catalog --> api
    codes --> api
    dsp --> api
    samples --> api
    validation --> api
    api --> downstream
```

## Read These First

- open [Module Map](module-map.md) first when the question is simply where a
  behavior lives
- open [Execution Model](execution-model.md) when the issue is whether a helper
  is still runtime-neutral
- open [Integration Seams](integration-seams.md) when the question crosses
  catalog, codes, DSP, and validation

## Pages In This Section

- [Module Map](module-map.md)
- [Dependency Direction](dependency-direction.md)
- [Execution Model](execution-model.md)
- [State And Persistence](state-and-persistence.md)
- [Integration Seams](integration-seams.md)
- [Error Model](error-model.md)
- [Extensibility Model](extensibility-model.md)
- [Code Navigation](code-navigation.md)
- [Architecture Risks](architecture-risks.md)

## First Code Roots

- catalog source for signal identity and wavelength meaning
- code-family source for deterministic spreading and secondary-code behavior
- DSP source for reusable front-end, timing, replica, spectrum, and tracking
  primitives
- raw-IQ and sample source for capture metadata and conversion contracts
- observation-validation source for signal-level compatibility checks
- error source for signal-owned failure language

## First Proof Check

Inspect the [signal architecture guide](../../../crates/bijux-gnss-signal/docs/ARCHITECTURE.md),
[public API](../../../crates/bijux-gnss-signal/docs/PUBLIC_API.md), and
[signal test guide](../../../crates/bijux-gnss-signal/docs/TESTS.md) first.
Then move through the code roots above and the matching integration tests so
architectural claims stay anchored to the real signal boundary and proof
families.

## Leave This Section When

- leave for [Foundation](../foundation/) when the real issue is still package
  ownership rather than structure
- leave for [Interfaces](../interfaces/) when the question is what callers may
  rely on rather than how the code is partitioned
- leave for [Quality](../quality/) when the structure is clear and the next
  question is whether the proof surface is strong enough
