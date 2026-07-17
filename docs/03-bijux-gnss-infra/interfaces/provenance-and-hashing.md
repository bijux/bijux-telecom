---
title: Provenance and Hashing
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Provenance and Hashing

Hashing in infra is about repository evidence, not about scientific identity.

## Main Owned Helpers

- `hash_config`
- `git_hash`
- `git_dirty`
- `cpu_features`
- front-end provenance capture persisted alongside run footprints

## Why They Matter

These helpers make it possible to explain how a run was prepared and under
which repository and machine conditions it was executed.

## Boundary Rule

Infra-owned hashes describe provenance and reproducibility evidence. They should
not expand into a general cryptographic utility bucket or replace core identity
semantics.

## Protecting Proof

- `crates/bijux-gnss-infra/docs/HASHING.md`
- `crates/bijux-gnss-infra/src/hash/provenance.rs`
- `crates/bijux-gnss-infra/src/run_layout/provenance/front_end.rs`
