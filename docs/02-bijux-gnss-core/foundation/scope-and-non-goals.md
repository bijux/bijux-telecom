---
title: Scope and Non-Goals
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Scope and Non-Goals

`bijux-gnss-core` is dense, but it is not unlimited. The crate owns shared
meaning, not general convenience.

## In Scope

- canonical identities for constellations, satellites, signals, and support
  inventory
- strong units, coordinate types, and time-system conversions
- acquisition, tracking, observation, differencing, and navigation-solution
  records
- diagnostics, schema versioning, validation-report shape, and artifact
  envelopes

## Explicit Non-Goals

- raw sample transport, sample conversion, or DSP primitives
- filesystem layout, run history, dataset registry, or repository manifests
- orbit propagation strategy, atmospheric correction policy, PPP, RAIM, or RTK
  estimators
- receiver state machines, scheduling, ports, or runtime metrics
- operator command parsing and report rendering

## Scope Test

Ask three questions before adding anything here:

1. Would at least two downstream crates depend on this meaning directly?
2. Does it remain valid outside one runtime, one solver, or one persistence
   layout?
3. Is the main value semantic stability rather than behavioral convenience?

If any answer is no, the stronger default is usually a downstream owner crate.

## First Proof Check

- `crates/bijux-gnss-core/docs/BOUNDARY.md`
- `crates/bijux-gnss-core/docs/CONTRACTS.md`
- `crates/bijux-gnss-core/docs/CHANGE_RULES.md`
