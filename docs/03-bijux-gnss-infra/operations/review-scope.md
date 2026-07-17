---
title: Review Scope
audience: mixed
type: explanation
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Review Scope

Review infra changes with a wider repository lens than the diff alone.

## Always Ask

- did this change alter dataset interpretation
- did persisted run-footprint meaning change
- did an override or sweep rule become more implicit rather than more typed
- did infra absorb behavior that belongs in command, receiver, signal, nav, or
  core
- would a reader still know where repository ownership begins and ends

## Raise The Review Bar For

- `src/run_layout.rs`
- `src/run_layout/`
- `src/datasets/`
- `src/artifact_inspection/`
- `src/validate_reference.rs`
- any change that rewrites manifest, report, or history semantics
