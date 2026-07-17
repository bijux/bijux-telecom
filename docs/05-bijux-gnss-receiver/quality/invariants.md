---
title: Invariants
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Invariants

These are the expectations that should remain true even as the receiver crate
grows.

## Boundary Invariants

- receiver runtime stays separate from command workflow policy
- repository persistence and indexing do not leak into runtime artifacts or
  ports
- public imports remain organized by runtime role through `api.rs`

## Runtime Invariants

- stage ordering and typed handoff remain explicit
- lower-owner signal and nav science are consumed, not silently re-owned
- artifacts represent in-memory runtime outputs before persistence
- validation and synthetic helpers remain proof surfaces for receiver behavior,
  not generic tooling buckets

## Review Invariant

If a change moves public runtime meaning, the docs and proof obligations must
move with it.

## First Proof Check

Inspect `crates/bijux-gnss-receiver/docs/PUBLIC_API.md`,
`crates/bijux-gnss-receiver/docs/RUNTIME.md`,
`crates/bijux-gnss-receiver/docs/PIPELINE.md`, and the most relevant runtime
integration tests to confirm the invariants named here still have checked
runtime proof.
