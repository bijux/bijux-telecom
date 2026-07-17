---
title: Dataset Contracts
audience: mixed
type: explanation
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Dataset Contracts

Dataset contracts are where repository files become typed capture metadata
instead of ad hoc path handling.

## Main Owned Records And Helpers

- `DatasetRegistry`
- `DatasetEntry`
- `RecordedCaptureProvenance`
- `load_raw_iq_metadata`
- `resolve_raw_iq_metadata`
- `parse_ecef`

## Why They Matter

These contracts let commands, tests, and tooling share one interpretation of
registry files, sidecars, and capture provenance instead of inventing slightly
different repository behavior in each caller.

## Boundary Rule

Infra owns the repository interpretation of dataset state. It does not own
signal-layer sample semantics or receiver execution over those datasets.

## Protecting Proof

- `crates/bijux-gnss-infra/docs/DATASETS.md`
- `crates/bijux-gnss-infra/src/datasets/`
