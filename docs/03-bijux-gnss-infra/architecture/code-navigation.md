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
   `datasets/`, `run_layout/`, `artifact_inspection/`, `overrides/`,
   `sweep.rs`, `hash/`, or `validate_reference.rs`
3. read the corresponding crate-local docs under
   `crates/bijux-gnss-infra/docs/`
4. confirm with the narrow infra tests and any relevant root handbook page

## Review Shortcut

If a change touches `run_layout/`, dataset registry parsing, or validation
adapters, review it as a repository contract change first and an
implementation-detail change second.

## First Proof Check

- `crates/bijux-gnss-infra/src/api.rs`
- `crates/bijux-gnss-infra/src/run_layout/`
- `crates/bijux-gnss-infra/tests/integration_overrides.rs`
