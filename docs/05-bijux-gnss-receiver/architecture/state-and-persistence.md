---
title: State And Persistence
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# State And Persistence

`bijux-gnss-receiver` owns a large amount of runtime state, but very little
repository persistence policy.

## State It Owns

- receiver configuration and derived pipeline configuration
- runtime logger, trace, metrics, and capture-start controls
- acquisition caches, candidate state, and ranking evidence
- tracking channel state, session artifacts, and lock evidence
- observation timing, smoothing, residual, and quality state
- in-memory `RunArtifacts` and validation-report structures
- synthetic scenario and runtime proof state

## Persistence It Does Not Own

- dataset or fixture discovery policies outside runtime adapters
- manifest naming, report paths as repository contracts, or history indexing
- repository directory structure and long-term run identity

## Why The Distinction Matters

Runtime state is allowed to be rich and execution-oriented. That does not grant
permission to smuggle repository semantics into the receiver just because some
state is later persisted elsewhere.

## Closest Proof

- `crates/bijux-gnss-receiver/src/engine/runtime.rs`
- `crates/bijux-gnss-receiver/src/artifacts.rs`
- `crates/bijux-gnss-receiver/src/validation_report.rs`
