---
title: Scope and Non-Goals
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Scope and Non-Goals

`bijux-gnss-infra` owns repository-facing infrastructure, not product behavior
that merely touches repository files.

## In Scope

- dataset registry records, sidecars, capture metadata, and coordinate parsing
- run directory identity, manifests, reports, history append semantics, and
  artifact headers for persisted outputs
- typed profile overrides, sweep expansion, and experiment specs
- provenance hashing and repository-state capture
- persisted artifact inspection and validation adapters

## Explicit Non-Goals

- DSP implementations or raw sample math
- orbit, atmosphere, PPP, RTK, or other navigation algorithms
- receiver stage orchestration and channel policy
- operator command parsing, rendering, or workflow wording

## Scope Test

Ask three questions before adding anything here:

1. Is the main concern repository state, persistence, or repeatability?
2. Would multiple callers need one shared interpretation of that repository
   concern?
3. Can the behavior stay typed without importing runtime or solver policy?

If any answer is no, a stronger owner probably exists elsewhere.

## First Proof Check

- `crates/bijux-gnss-infra/docs/BOUNDARY.md`
- `crates/bijux-gnss-infra/docs/CONTRACTS.md`
