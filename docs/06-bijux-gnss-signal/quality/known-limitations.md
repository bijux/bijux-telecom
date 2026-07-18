---
title: Known Limitations
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Known Limitations

`bijux-gnss-signal` can prove reusable signal behavior. It cannot prove that a
receiver will acquire, track, navigate, persist, or present that behavior
correctly. Readers should treat the crate's evidence as strong inside the
signal boundary and deliberately incomplete outside it.

## Limits Readers Should Know

| limitation | consequence | honest reading |
| --- | --- | --- |
| Signal proof is not receiver proof. | Correct code generation or spectrum behavior does not guarantee acquisition lock, tracking continuity, or navigation accuracy. | Pair signal changes with receiver tests only when the claim crosses into runtime behavior. |
| Reference data is persuasive but narrow. | A checked-in reference can hide assumptions about constellation, component, chip order, sampling rate, or front-end bandwidth. | Name the reference family and the physical assumption it defends. |
| Public metadata is widely consumed. | A small catalog field change can alter receiver search, artifact metadata, or documentation claims downstream. | Treat catalog and wavelength changes as cross-crate contract changes. |
| DSP helpers are reusable by design. | Adding scheduling, persistence, or operator policy here creates false ownership. | Keep this crate pure from a product perspective; move policy to the owner that executes it. |
| The facade is intentionally curated. | Exposing internal tables or helper paths can make implementation detail look stable. | Use `api.rs` and `PUBLIC_API.md` as the compatibility boundary. |

## Claim Boundaries

This handbook supports claims such as:

- "Galileo E1 component metadata is registered consistently."
- "The CBOC spectrum helper preserves the modeled signal shape."
- "Long-duration replica generation is chunk-stable."
- "Raw-IQ sample conversion preserves documented numeric meaning."

It does not support claims such as:

- "The receiver will lock every supported signal under field conditions."
- "The navigation stack remains accurate under every estimator configuration."
- "Repository artifacts preserve this signal metadata after persistence."

## First Proof Route

Start with `crates/bijux-gnss-signal/docs/TESTS.md`, then select the proof
family by the claim being made:

- catalog and component claims:
  `crates/bijux-gnss-signal/tests/integration_signal_component_registry.rs`
- spectral claims:
  `crates/bijux-gnss-signal/tests/integration_signal_spectrum_cboc.rs`
- code-generation claims:
  the constellation-specific reference test under
  `crates/bijux-gnss-signal/tests/`
- conversion claims:
  `crates/bijux-gnss-signal/tests/integration_iq_sample_conversion.rs`

If a document sentence needs stronger proof than these files provide, narrow
the sentence or route the claim to the owning crate.
