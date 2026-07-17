---
title: API Surface
audience: mixed
type: explanation
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# API Surface

`bijux-gnss-infra` publishes one deliberate downstream surface:
`bijux_gnss_infra::api`.

## Public Module Policy

- `src/api.rs` is the curated infrastructure surface
- infra-owned helpers live here alongside a small number of lower-level
  re-exports used for repository-facing convenience
- re-exports are justified only when they strengthen the repository boundary
  instead of bypassing it

## Why The Policy Exists

Without a curated surface, callers would either rebuild repository logic
themselves or depend on internal module paths that make refactoring expensive
and ownership blurry.

## Protecting Proof

- `crates/bijux-gnss-infra/src/api.rs`
- `crates/bijux-gnss-infra/README.md`
