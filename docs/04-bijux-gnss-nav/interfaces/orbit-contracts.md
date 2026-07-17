---
title: Orbit Contracts
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# Orbit Contracts

Orbit contracts define the typed state that downstream solvers and correction
families consume after navigation products have been interpreted.

## Owned Orbit Surfaces

- constellation-specific ephemeris records
- broadcast and precise satellite-state helpers
- satellite uncertainty and provider seams
- position-solver navigation wrappers that organize orbit state for estimation

## Caller Expectations

- orbit records should reflect scientific meaning, not repository transport
- provider seams should let callers supply product-backed state without
  redefining the contract
- consumers should not need to know internal file layout to use orbit state

## Closest Proof

- `crates/bijux-gnss-nav/src/orbits/`
- `crates/bijux-gnss-nav/docs/ORBITS.md`
