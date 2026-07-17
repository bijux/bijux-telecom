---
title: Scope And Non-Goals
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Scope And Non-Goals

## Scope

`bijux-gnss-signal` owns:

- canonical signal registry entries and wavelength conversion helpers
- spreading-code, subcarrier, and secondary-code behavior for supported GNSS
  families
- local-code, NCO, replica, front-end, spectrum, and tracking-loop primitives
  that are usable outside one receiver pipeline
- explicit raw-IQ metadata and controlled sample-quantization helpers
- signal-layer observation compatibility and epoch-shape validation

## Non-Goals

`bijux-gnss-signal` does not own:

- channel scheduling, stage composition, or emitted run artifacts
- file-path policy, sidecar resolution, or persisted dataset structure
- orbit products, estimator convergence, PPP, RTK, or solution acceptance
- operator-facing command semantics, report wording, or workflow sequencing

## Practical Rule

If a helper needs to know where samples came from on disk, how a receiver stage
is sequenced, or whether a final navigation answer is trustworthy, the helper
is already outside this crate's proper scope.
