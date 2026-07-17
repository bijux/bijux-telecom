---
title: State and Serialization
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# State and Serialization

`bijux-gnss-core` carries little runtime state, but it carries a large amount
of persisted meaning.

## State Model

- most families are value types, record types, or validation traits
- runtime scheduling and mutable orchestration live elsewhere
- the important state question here is whether serialized records preserve
  scientific meaning without needing implementation context

## Serialization Responsibilities

- artifact envelopes and payload versions live in `src/artifact.rs` and
  `src/artifact/`
- observation and differencing records live in `src/observation.rs` and
  `src/observation/`
- navigation-solution records live in `src/nav_solution.rs`
- diagnostics and validation reports must stay structured enough for downstream
  tools to reason about them through `src/config.rs`, `src/diagnostic/`, and
  related docs

## Persistence Boundary

Core owns record meaning and validation rules. It does not own file names,
directory layout, history append policy, or export workflows. Those belong to
`bijux-gnss-infra` and the command boundary.

## First Proof Check

- `crates/bijux-gnss-core/docs/SERIALIZATION.md`
- `crates/bijux-gnss-core/src/artifact/`
- `crates/bijux-gnss-core/src/config.rs`
- `crates/bijux-gnss-core/tests/nav_artifact_validation.rs`
- `crates/bijux-gnss-core/tests/tracking_artifact_validation.rs`
