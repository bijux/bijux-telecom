---
title: Package Overview
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Package Overview

`bijux-gnss-receiver` is the runtime owner for GNSS receiver execution in
`bijux-telecom`.

## One-Sentence Role

This crate turns configured inputs, runtime sinks, and stage engines into a
concrete receiver run that emits in-memory artifacts and runtime-side
validation evidence.

## What Readers Should Remember

- this crate composes lower-level scientific owners instead of replacing them
- stage ordering, runtime state, and side-effect seams belong here
- persisted naming, manifests, and repository history do not belong here

## Major Runtime Families

- `src/engine/` owns configuration, validation, runtime state, logging,
  metrics, support-matrix handling, and top-level receiver composition
- `src/pipeline/` owns acquisition, tracking, observations, optional
  navigation, and handoff reporting between stages
- `src/ports/` and `src/io/` own the runtime-facing source, sink, and clock
  seams
- `src/artifacts.rs`, `src/reference_validation.rs`,
  `src/covariance_realism.rs`, `src/validation_helpers.rs`, and
  `src/validation_report.rs` own receiver-boundary outputs and runtime-side
  validation helpers
- `src/sim/` owns synthetic signal generation and receiver-boundary execution
  used for tests and validation

## Why This Package Is Heavy

The receiver boundary is large because orchestration is not a thin wrapper. It
has to bind configuration, stage budgets, diagnostics, ports, stage outputs,
and optional navigation behavior into one runtime that other crates can trust.

## Closest Code Proof

- `crates/bijux-gnss-receiver/src/api.rs`
- `crates/bijux-gnss-receiver/src/engine/receiver_config.rs`
- `crates/bijux-gnss-receiver/src/engine/engine.rs`
- `crates/bijux-gnss-receiver/src/pipeline/mod.rs`
- `crates/bijux-gnss-receiver/src/validation_report.rs`
- `crates/bijux-gnss-receiver/src/sim/synthetic.rs`
