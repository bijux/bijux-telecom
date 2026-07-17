---
title: Review Checklist
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Review Checklist

Use this checklist when reviewing a `bijux-gnss-signal` change.

## Checklist

- is the owning signal surface named clearly
- does the change preserve the signal boundary against receiver, repository, or
  navigation creep
- if `src/api.rs` changed, was the public contract reviewed explicitly
- if a code family changed, does the matching reference proof still read as
  canonical
- if a DSP helper changed, does the proof cover continuity or runtime-neutral
  behavior honestly
- if reference data changed, is the reason technically credible
- do the handbook pages still match the changed behavior

## First Proof Check

Inspect `crates/bijux-gnss-signal/docs/PUBLIC_API.md`,
`crates/bijux-gnss-signal/docs/BOUNDARY.md`, and
`crates/bijux-gnss-signal/docs/TESTS.md` first. Then read the changed module
and its specific proof family so the checklist is answered with source-backed
evidence rather than with generic confidence.
