---
title: Signal Model Assumptions
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-18
---

# Signal Model Assumptions

Signal assumptions are reusable only when they are backed by code, reference
catalogs, and tests in `bijux-gnss-signal`. A downstream crate may consume these
assumptions for acquisition, tracking, synthetic generation, validation, or
documentation, but it must not silently broaden them.

## Assumption Route

```mermaid
flowchart LR
    catalog["signal catalog"]
    code["code family<br/>reference catalogs"]
    dsp["sample and replica<br/>DSP helpers"]
    proof["signal tests"]
    consumers["receiver command infra<br/>nav and testkit consumers"]

    catalog --> proof
    code --> proof
    dsp --> proof
    proof --> consumers
```

## Current Assumptions

| assumption | signal-owned proof | downstream limit |
| --- | --- | --- |
| GPS L1 C/A has a 1023-chip primary period for PRNs 1 through 32 | C/A reference, period, correlation, and long-duration tests | receiver lock behavior and navigation data interpretation remain downstream |
| sampled-code generation advances by code-rate to sample-rate progression | fractional sampling and long-duration continuity tests | callers cannot assume rounded samples-per-chip behavior |
| carrier helpers interpret Doppler relative to configured intermediate frequency | carrier wipeoff, NCO, acquisition, and synthetic reference tests | receiver search windows decide how much Doppler to inspect |
| deterministic GPS navigation sign modulation uses a 50 bps data rate when enabled | synthetic navigation-bit and replica tests | nav message semantics are not proven by this signal assumption |
| Galileo E1 code and component references are modeled for acquisition-oriented use | E1B, E1C, registry, spectrum, and synthetic tests | tracking policy and ambiguity resolution remain receiver decisions |
| registry component roles define reusable signal identity | component registry tests | public API users still need receiver docs for stage behavior |

## Change Rules

- Add an assumption only when a named reference test or catalog anchors it.
- Name the signal family, component, rate, or modulation detail explicitly.
- Update receiver or command docs only after the signal assumption exists.
- Do not use this page to promise full receiver support for a signal family.

## First Proof Check

Inspect the [signal catalog guide](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-signal/docs/CATALOG.md),
[code family guide](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-signal/docs/CODE_FAMILIES.md),
and [DSP guide](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-signal/docs/DSP.md). Then inspect
catalog source and focused signal tests for C/A reference behavior, long-duration
C/A chunks, Galileo E1B reference behavior, and GPS L2C replica modeling.
