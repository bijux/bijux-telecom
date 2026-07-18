---
title: Ownership Boundary
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Ownership Boundary

The most important discipline in `bijux-gnss-signal` is keeping reusable signal
behavior separate from operational policy.

## This Crate Owns

- what a supported signal is
- how its code or subcarrier is generated and sampled
- how samples are normalized, quantized, or represented
- how reusable front-end, spectrum, replica, and tracking math behaves
- whether a set of observations is signal-compatible at the band and lock-state
  level

## Neighbor Crates Own The Rest

- [Core](../../bijux-gnss-core/foundation/package-overview.md) owns
  cross-package types, units, identifiers, and observation record shapes
- [Receiver](../../bijux-gnss-receiver/foundation/package-overview.md) owns
  runtime composition, ports, stage sequencing, and receiver artifacts
- [Navigation](../../bijux-gnss-nav/foundation/package-overview.md) owns
  navigation-domain interpretation and estimator science
- [Infra](../../bijux-gnss-infra/foundation/package-overview.md) owns
  repository-side dataset and run-layout persistence
- [Command](../../bijux-gnss/foundation/package-overview.md) owns operator
  commands, workflows, and top-level reports

## Boundary Test

When a change proposal says "this needs to live near signal math," that is not
enough. The real question is whether the behavior is still reusable without
assuming one receiver runtime, one repository layout, or one operator workflow.

## First Proof Check

Read the signal [boundary guide](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-signal/docs/BOUNDARY.md)
and [contract guide](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-signal/docs/CONTRACTS.md)
first. Then inspect the [public API facade](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-signal/src/api.rs),
[signal catalog](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-signal/src/catalog.rs),
[DSP source](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-signal/src/dsp/mod.rs),
[raw-IQ model](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-signal/src/raw_iq.rs), and
[observation validation source](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-signal/src/obs_validation.rs)
to confirm the crate still owns reusable signal contracts rather than runtime,
persistence, or operator policy.
