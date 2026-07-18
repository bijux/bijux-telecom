---
title: GPS L1 C/A Reference
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-18
---

# GPS L1 C/A Reference

Use this page when the reader needs the stable GPS L1 C/A signal profile before
opening code, tests, or receiver behavior. The signal crate owns these facts;
the receiver crate owns acquisition, tracking, and lock policy built on them.

## Stable Signal Facts

| fact | value | repository meaning |
| --- | --- | --- |
| constellation | GPS | shared identity is `Constellation::Gps` |
| band | L1 | shared identity is `SignalBand::L1` |
| code | C/A | shared identity is `SignalCode::Ca` |
| carrier frequency | 1575.42 MHz | catalog and wavelength helpers derive carrier-dependent values |
| code rate | 1.023 MHz | sampling and replica helpers use this as the nominal chip rate |
| primary-code length | 1023 chips | one primary period is one complete C/A sequence |
| primary-code period | 1 ms | receiver stages should not infer longer data-bit behavior from this page |

## Ownership Route

```mermaid
flowchart LR
    facts["GPS L1 C/A facts"]
    code["ca_code.rs<br/>chips and sampling"]
    catalog["catalog.rs<br/>signal identity"]
    dsp["dsp replica helpers"]
    receiver["bijux-gnss-receiver<br/>acquisition and tracking"]

    facts --> code
    facts --> catalog
    code --> dsp
    catalog --> receiver
    dsp --> receiver
```

## What This Page Proves

- The C/A sequence is a 1023-chip primary code with a 1 ms nominal period.
- Public signal identity must route through the registry instead of hard-coded
  caller assumptions.
- Replica and sampling behavior belongs in signal only while it remains reusable
  substrate.
- Receiver defaults are downstream choices and must be proven in receiver docs,
  receiver tests, or emitted receiver artifacts.

## When To Leave This Page

| reader question | better owner |
| --- | --- |
| How is the C/A sequence generated or sampled? | `crates/bijux-gnss-signal/src/codes/ca_code.rs` |
| How is the signal exposed to other crates? | `crates/bijux-gnss-signal/src/catalog.rs` and `src/api.rs` |
| How does an acquisition search use this signal? | `bijux-gnss-receiver` acquisition docs and tests |
| How does tracking interpret phase, lock, or CN0? | `bijux-gnss-receiver` tracking and diagnostics docs |
| How does navigation use decoded GPS data? | `bijux-gnss-nav` |

## First Proof Check

Inspect `crates/bijux-gnss-signal/src/codes/ca_code.rs`,
`crates/bijux-gnss-signal/src/catalog.rs`,
`crates/bijux-gnss-signal/tests/integration_ca_code_reference.rs`,
`crates/bijux-gnss-signal/tests/integration_ca_code_period_length.rs`, and
`crates/bijux-gnss-signal/tests/integration_ca_code_long_duration_phase.rs`.
