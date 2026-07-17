---
title: State And Persistence
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# State And Persistence

`bijux-gnss-nav` owns a large amount of typed scientific state, but very little
repository persistence policy.

## State It Owns

- decoded navigation records and precise-product provider state
- orbit, clock, bias, and correction evidence structures
- solver state for EKF, PPP, and RTK families
- solution, refusal, and integrity evidence records
- GNSS-specific time conversions and offset evidence

## Persistence It Does Not Own

- dataset or fixture discovery rules
- artifact naming or run-directory layout
- report placement and repository history policy

## Why The Distinction Matters

PPP and RTK filters can be stateful without owning persistence policy. A parsed
SP3 provider can hold cached scientific truth without deciding where the file
came from or where downstream evidence is stored.

## Closest Proof

- `crates/bijux-gnss-nav/src/formats/precise_products/mod.rs`
- `crates/bijux-gnss-nav/src/estimation/ppp/config.rs`
- `crates/bijux-gnss-nav/src/estimation/rtk/ambiguity.rs`
- `crates/bijux-gnss-nav/src/time.rs`
