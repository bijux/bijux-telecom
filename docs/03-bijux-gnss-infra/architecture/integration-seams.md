---
title: Integration Seams
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Integration Seams

Infra’s seams matter because it touches many owners while trying not to become
one of them.

## Main Seams

- dataset registry and sidecars are the seam between repository files and
  typed metadata through `src/datasets/registry.rs` and
  `src/datasets/raw_iq_metadata.rs`
- run layout is the seam between execution context and persisted footprint
  through `src/run_layout.rs` and the nested `src/run_layout/` families
- override and sweep logic are the seam between maintained configuration and
  variant expansion
- artifact inspection is the seam between produced artifacts and later review
  through `src/artifact_inspection/` and its validation subfamily
- validation adapters are the seam between persisted evidence and reference
  comparison workflows

## Why These Seams Matter

Weak seams here create duplicated repository logic in commands, tests, and
tooling. Strong seams keep repository behavior typed and reusable.

## First Proof Check

- `crates/bijux-gnss-infra/src/datasets/`
- `crates/bijux-gnss-infra/src/run_layout.rs`
- `crates/bijux-gnss-infra/src/run_layout/`
- `crates/bijux-gnss-infra/src/artifact_inspection/`
- `crates/bijux-gnss-infra/src/validate_reference.rs`
