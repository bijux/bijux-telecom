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

## First Proof Check

Use `crates/bijux-gnss-core/docs/PUBLIC_API.md`,
`crates/bijux-gnss-core/docs/CONTRACTS.md`, and
`crates/bijux-gnss-core/docs/TESTS.md` as the review map. Then inspect the
changed source family and its matching tests so review depth follows contract
risk rather than diff size alone.
