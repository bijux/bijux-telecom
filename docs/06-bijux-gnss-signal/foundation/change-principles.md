---
title: Change Principles
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Change Principles

Changes in `bijux-gnss-signal` travel far because many crates trust its
behavior. The right standard is conservative structure with explicit proof.

## Principles

- prefer one canonical implementation of a signal rule over parallel near-miss
  helpers
- keep reusable math separate from orchestration and persistence concerns
- treat raw-IQ and sample contracts as public boundaries, not convenience
  details
- make signal-family additions explicit by constellation and band instead of
  hiding them behind generic buckets
- preserve deterministic behavior where reference data and continuity tests
  rely on it

## Anti-Patterns

- moving runtime-specific decisions into a signal helper because a receiver path
  already needs the data
- adding a public export before deciding whether the signal crate should own it
- weakening reference or continuity proof because the implementation "looks
  equivalent"
