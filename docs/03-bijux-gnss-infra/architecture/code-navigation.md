---
title: Code Navigation
audience: mixed
type: explanation
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Code Navigation

Use this route when you need to inspect infra quickly.

## Fast Reading Path

1. start at `src/api.rs` to see the curated infrastructure surface
2. jump to the owning family:
   `src/datasets/registry.rs`, `src/datasets/raw_iq_metadata.rs`,
   `src/run_layout.rs`, `src/artifact_inspection/`, `src/overrides/`,
   `src/sweep.rs`, `src/hash/`, or `src/validate_reference.rs`
3. read the corresponding crate-local docs under
   `crates/bijux-gnss-infra/docs/`
4. confirm with the narrow infra tests and any relevant root handbook page

## Review Shortcut

If a change touches `src/run_layout.rs`, `src/run_layout/`, dataset registry
parsing, or validation adapters, review it as a repository contract change
first and an implementation-detail change second.

## First Proof Check

- `crates/bijux-gnss-infra/src/api.rs`
- `crates/bijux-gnss-infra/src/datasets/registry.rs`
- `crates/bijux-gnss-infra/src/datasets/raw_iq_metadata.rs`
- `crates/bijux-gnss-infra/src/run_layout.rs`
- `crates/bijux-gnss-infra/src/run_layout/`
- `crates/bijux-gnss-infra/tests/integration_overrides.rs`
