---
title: Domain Language
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Domain Language

The core crate succeeds when downstream packages can speak one GNSS language
without re-explaining basics to each other.

## Vocabulary Families

- identity language:
  constellation, satellite, signal, band, code, and support-matrix status
- time language:
  GPS time, UTC, TAI, leap seconds, receiver sample traces, and epoch framing
- physical language:
  meters, seconds, cycles, hertz, chips, and coordinate frames
- measurement language:
  acquisition results, tracking epochs, observation epochs, differencing
  records, and navigation solutions
- artifact language:
  versioned envelopes, payload kinds, validation reports, and diagnostics

## Why This Page Exists

The crate carries many types that look simple in isolation. The problem is not
understanding one type. The problem is preserving one shared vocabulary across
signal, navigation, receiver, and infrastructure work.

## Naming Pressure

If a downstream crate invents a near-synonym for an existing core record, the
reader loses more than naming clarity. They lose the ability to trust cross-
crate comparisons and persisted artifacts.

## First Proof Check

- `crates/bijux-gnss-core/src/ids.rs`
- `crates/bijux-gnss-core/src/time.rs`
- `crates/bijux-gnss-core/src/units.rs`
- `crates/bijux-gnss-core/src/observation/`
- `crates/bijux-gnss-core/src/nav_solution.rs`
