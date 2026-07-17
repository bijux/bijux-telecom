---
title: Extensibility Model
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Extensibility Model

`bijux-gnss-core` should extend by deepening existing contract families more
often than by creating new ones.

## Preferred Extension Paths

- extend `src/observation/` when a new shared measurement record is genuinely
  needed
- extend `src/artifact/` with explicit version boundaries rather than rewriting
  existing payload meaning
- extend `src/api.rs` only after the shared contract need is proven
- extend docs, invariants, and protecting tests in the same change set

## Extension Smells

- new top-level module for one small helper
- public export added before two downstream crates actually need it
- runtime or persistence assumptions embedded in a supposedly generic record

## First Proof Check

- `crates/bijux-gnss-core/docs/CHANGE_RULES.md`
- `crates/bijux-gnss-core/tests/public_api_guardrail.rs`
