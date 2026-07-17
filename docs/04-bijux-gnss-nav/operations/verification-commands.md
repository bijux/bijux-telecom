---
title: Verification Commands
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# Verification Commands

Use the narrowest relevant command that still proves the changed scientific
surface.

## Focused Commands From Repository Root

```sh
cargo test -p bijux-gnss-nav --test integration_position
cargo test -p bijux-gnss-nav --test integration_precise_products
cargo test -p bijux-gnss-nav --test integration_rtk_baseline_accuracy
```

## Additional Targeted Checks Worth Using

```sh
cargo test -p bijux-gnss-nav --test integration_broadcast_orbit_reference
cargo test -p bijux-gnss-nav --test integration_sp3
cargo test -p bijux-gnss-nav --test integration_time_system_conversions
```

## Command Choice Rule

- use parser or product tests for format changes
- use orbit and time reference tests for state-interpretation changes
- use correction-specific tests before broad position tests when possible
- use PPP or RTK family tests before wider mixed-navigation tests

## First Proof Check

Use `crates/bijux-gnss-nav/docs/TESTS.md`,
`crates/bijux-gnss-nav/docs/ESTIMATION.md`,
`crates/bijux-gnss-nav/docs/FORMATS.md`, and
`crates/bijux-gnss-nav/docs/TIME.md` as the verification map. Then inspect the
changed source family so the proof set stays aligned with the actual scientific
surface that moved.
