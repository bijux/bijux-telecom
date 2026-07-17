---
title: Code Navigation
audience: mixed
type: explanation
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Code Navigation

When you need to inspect the crate quickly, use this route order.

## Fast Reading Path

1. start at `src/lib.rs` to confirm the crate exports one deliberate module:
   `api`
2. read `src/api.rs` to see what the crate claims publicly
3. jump to the owning family:
   `artifact/`, `observation/`, `nav_solution.rs`, `ids.rs`, `time.rs`,
   `units.rs`, `geo.rs`, `diagnostic/`, or `config.rs`
4. read the corresponding crate-local docs under `crates/bijux-gnss-core/docs/`
5. confirm with the narrow integration or property test that protects the
   contract

## Review Shortcut

If a change touches `src/api.rs`, `src/artifact/`, `src/observation/`, or
`src/time.rs`, treat it as a cross-crate contract review first and an
implementation review second.

## First Proof Check

- `crates/bijux-gnss-core/src/lib.rs`
- `crates/bijux-gnss-core/src/api.rs`
- `crates/bijux-gnss-core/tests/public_api_guardrail.rs`
