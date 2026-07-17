---
title: Integration Seams
audience: mixed
type: explanation
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Integration Seams

Infra’s seams matter because it touches many owners while trying not to become
one of them.

## Main Seams

- dataset registry and sidecars are the seam between repository files and
  typed metadata
- run layout is the seam between execution context and persisted footprint
- override and sweep logic are the seam between maintained configuration and
  variant expansion
- artifact inspection is the seam between produced artifacts and later review
- validation adapters are the seam between persisted evidence and reference
  comparison workflows

## Why These Seams Matter

Weak seams here create duplicated repository logic in commands, tests, and
tooling. Strong seams keep repository behavior typed and reusable.

## First Proof Check

- `crates/bijux-gnss-infra/src/datasets/`
- `crates/bijux-gnss-infra/src/run_layout/`
- `crates/bijux-gnss-infra/src/artifact_inspection/`
- `crates/bijux-gnss-infra/src/validate_reference.rs`
