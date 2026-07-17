---
title: Dependencies And Adjacencies
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# Dependencies And Adjacencies

This page explains which nearby crates `bijux-gnss-nav` depends on directly
and which ones sit beside it as boundary checks.

## Direct Technical Dependencies

- `bijux-gnss-core` for shared artifacts, time primitives, observation records,
  and signal identifiers
- `bijux-gnss-signal` where navigation logic needs signal-domain identifiers
  and semantics but not raw processing ownership

## Adjacencies That Matter More Than The Cargo Graph

- `bijux-gnss-receiver` is the main consumer whose orchestration pressure can
  tempt scientific drift if boundaries are not explicit
- `bijux-gnss-infra` validates persisted runs against navigation truth and can
  tempt file-discovery logic into nav if left unchecked
- `bijux-gnss` exposes operator-visible commands and therefore depends on a
  stable public navigation vocabulary

## Practical Reading Rule

If a change needs simultaneous edits in nav and one neighboring crate, review
the boundary first. Cross-crate edits are often a sign that ownership is
unclear even when the code still compiles.
