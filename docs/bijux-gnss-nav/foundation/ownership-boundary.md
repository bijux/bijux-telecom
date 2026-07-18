---
title: Ownership Boundary
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# Ownership Boundary

`bijux-gnss-nav` sits above `bijux-gnss-core` and `bijux-gnss-signal`, and
below `bijux-gnss`, `bijux-gnss-receiver`, and parts of `bijux-gnss-infra`.

## Upstream Dependencies It May Assume

- `bijux-gnss-core` provides shared artifact, time, signal, and observation
  contracts that are not specific to one navigation method
- `bijux-gnss-signal` provides signal-domain identifiers and lower-level signal
  semantics the navigation layer may reference but does not reinterpret as raw
  DSP behavior

## Downstream Packages It Serves

- `bijux-gnss-receiver` consumes navigation state and solver behavior while
  keeping stage scheduling and live orchestration outside this crate
- `bijux-gnss` consumes the public API to expose commands and reports
- `bijux-gnss-infra` consumes navigation truth when validating persisted
  evidence against reference products

## Boundary Tests

- if a concern starts from bytes or lines that encode navigation truth, it can
  belong here
- if a concern starts from how a receiver loops over samples, it does not
  belong here
- if a concern starts from where files live on disk, it does not belong here
- if a concern changes the meaning of pseudorange corrections, orbit
  propagation, or ambiguity resolution, it does belong here

## Where Boundary Drift Usually Starts

- solver result packaging begins to carry repository layout policy
- precise-product parsing begins to carry file discovery or operator defaults
- runtime wrappers duplicate scientific thresholds instead of depending on the
  public navigation surface
