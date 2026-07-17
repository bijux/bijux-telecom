---
title: Execution Model
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Execution Model

Infra execution is mostly about preparing, interpreting, and validating
repository state around product runs.

## Typical Flow

1. resolve a dataset or raw-IQ metadata source
2. prepare a run through typed identity, layout, and artifact-header helpers
3. apply typed overrides or expand a sweep when needed
4. persist manifests, reports, and history entries
5. inspect or validate persisted artifacts later through infrastructure entry
   points

## Why This Is Not A Product Pipeline

The work here surrounds product execution rather than performing signal or
navigation computation itself. Infra’s execution model is orchestration of
repository state, not GNSS stage math.

## First Proof Check

- `crates/bijux-gnss-infra/src/commands.rs`
- `crates/bijux-gnss-infra/src/datasets/`
- `crates/bijux-gnss-infra/src/run_layout/`
- `crates/bijux-gnss-infra/src/overrides/`
- `crates/bijux-gnss-infra/src/sweep.rs`
- `crates/bijux-gnss-infra/src/artifact_inspection/`
