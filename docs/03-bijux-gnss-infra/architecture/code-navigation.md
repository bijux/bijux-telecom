---
title: Code Navigation
audience: mixed
type: architecture
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
   `src/run_layout.rs` and `src/run_layout/identity.rs`,
   `src/artifact_inspection/` and `src/artifact_inspection/validation.rs`,
   `src/overrides/receiver_profile.rs`, `src/experiments.rs`, `src/sweep.rs`,
   `src/hash/provenance.rs`, or `src/validate_reference.rs`
3. read the corresponding crate-local docs under
   `crates/bijux-gnss-infra/docs/`, especially `PUBLIC_API.md`,
   `DATASETS.md`, `RUN_LAYOUT.md`, `OVERRIDES.md`, `HASHING.md`,
   `VALIDATION.md`, and `TESTS.md`
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
- `crates/bijux-gnss-infra/tests/integration_guardrails.rs`
- `crates/bijux-gnss-infra/tests/integration_overrides.rs`
