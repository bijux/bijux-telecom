---
title: Architecture
audience: mixed
type: index
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Architecture

Open this section when the question is structural: where datasets, run layout,
artifact inspection, overrides, hashing, and validation adapters live in code,
and how the crate avoids becoming a generic glue bucket.

## First Proof Check

- `crates/bijux-gnss-infra/src/datasets/`
- `crates/bijux-gnss-infra/src/run_layout/`
- `crates/bijux-gnss-infra/src/artifact_inspection/`
- `crates/bijux-gnss-infra/docs/ARCHITECTURE.md`

## Leave This Section When

- leave for [Foundation](../foundation/) when the real disagreement is still
  about ownership rather than structure
- leave for [Interfaces](../interfaces/) when the structural question is
  already about manifest or dataset contract shape
- leave for [Quality](../quality/) when the structure is clear and the next
  question is proof
