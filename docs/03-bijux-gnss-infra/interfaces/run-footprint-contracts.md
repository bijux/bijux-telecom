---
title: Run Footprint Contracts
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Run Footprint Contracts

Run footprint contracts define the durable repository record of an execution.

## Main Owned Records And Helpers

- `RunContextArgs`
- `RunDirectoryLayout`
- `RunManifest`
- `RunReport`
- `RunHistoryEntry`
- `run_dir`, `artifacts_dir`, `artifact_header`
- `write_manifest`, `write_run_report`, `append_run_history_entry`
- `run_report_schema_version`

## Why They Matter

These contracts are the repository’s memory of a run. They must stay readable
and stable after the command that created them is gone.

## Boundary Rule

Infra owns path resolution, manifest/report shape, and history append
semantics. It does not own the runtime-side content generation that receiver or
nav code performs before persistence.

## Protecting Proof

- `crates/bijux-gnss-infra/docs/RUN_LAYOUT.md`
- `crates/bijux-gnss-infra/src/run_layout.rs`
- `crates/bijux-gnss-infra/src/run_layout/records.rs`
- `crates/bijux-gnss-infra/src/run_layout/persistence.rs`
