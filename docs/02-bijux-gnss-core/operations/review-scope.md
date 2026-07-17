---
title: Review Scope
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Review Scope

Review `bijux-gnss-core` changes with a wider scope than the diff alone.

## Always Ask

- did this change expand the public surface
- did serialized meaning change
- did an invariant narrow or broaden
- did a stronger downstream owner exist
- did the docs move with the contract

## Raise The Review Bar For

- `src/api.rs`
- `src/artifact/`
- `src/time.rs`
- `src/observation/`
- anything touching fixture files or public validation behavior
