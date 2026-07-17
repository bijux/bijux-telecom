---
title: Local Development
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Local Development

Local development in `bijux-gnss-infra` should stay narrow and repository-
contract oriented.

## Good Development Loop

1. identify the owning contract family first
2. inspect the relevant source area such as `src/datasets/registry.rs`,
   `src/datasets/raw_iq_metadata.rs`, `src/run_layout.rs`,
   `src/run_layout/`, or `src/artifact_inspection/`, then read the matching
   crate-local docs
3. make the smallest coherent repository-contract change
4. run the narrowest protecting tests
5. only then widen into higher-level caller fallout if needed

## Why Narrow First

Broad workspace runs are often less useful than a well-chosen narrow check at
the dataset, run-layout, or override boundary being changed.
