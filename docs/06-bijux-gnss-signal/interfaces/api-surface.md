---
title: API Surface
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# API Surface

`bijux_gnss_signal::api` is the deliberate public facade. Downstream crates
should prefer it over reaching into private module layout.

## Main Public Families

- catalog and physical helpers for signal lookup, wavelength conversion, and
  default-acquisition selection
- code-family helpers for GPS, Galileo, BeiDou, and GLONASS signal generation
- DSP helpers for front-end response, local code, NCOs, replica generation,
  spectrum, and tracking math
- raw-IQ and sample helpers for metadata and quantization
- validation helpers for dual-frequency compatibility, inter-frequency
  alignment, and epoch checks
- trait seams for sample and correlation integration

## Public Trait Surface

The intentionally public traits are `SignalSource`, `SampleSource`,
`Correlator`, and `SampleSink`. They are signal-boundary seams, not a license
to move receiver orchestration into this crate.

## Architectural Rule

The facade is intentionally broad in capability but narrow in ownership. It
should publish reusable signal behavior without exposing internal structure as a
public promise.

## Protecting Proof

- `crates/bijux-gnss-signal/docs/PUBLIC_API.md`
- `crates/bijux-gnss-signal/docs/CATALOG.md`
- `crates/bijux-gnss-signal/docs/VALIDATION.md`
