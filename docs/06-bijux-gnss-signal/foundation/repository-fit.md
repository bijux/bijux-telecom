---
title: Repository Fit
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Repository Fit

`bijux-gnss-signal` sits in the middle of the GNSS stack. It is lower than
commands and receiver runtime, but higher than bare shared units and IDs.

## Upstream Inputs

- `bijux-gnss-core` provides shared types such as `SignalSpec`, `SatId`,
  `ObsEpoch`, sample buffer types, and physical units
- numeric dependencies such as `num-complex` and `rustfft` support the signal
  math, but do not define repository ownership

## Downstream Consumers

- `bijux-gnss-receiver` composes these signal primitives into staged runtime
  behavior
- `bijux-gnss-nav` may consume signal identifiers and helpers while remaining
  the owner of navigation science
- `bijux-gnss` and its validation flows use the public API without becoming
  alternate owners of signal behavior

## Why The Middle Layer Matters

Without this crate, the repository would either duplicate signal behavior
across higher-level packages or force `bijux-gnss-core` to absorb signal-domain
details that do not belong in a shared foundational crate.
