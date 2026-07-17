---
title: Validation Adapters
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Validation Adapters

Validation adapters are the repository-facing bridge between persisted
artifacts and validation/reference comparison flows.

## Main Owned Entry Points

- `validate_reference`
- selected re-exports of lower-level validation helpers through the curated
  infra API

## Why They Matter

These adapters keep repository workflows from re-implementing alignment and
comparison setup while still leaving runtime-side validation ownership in the
receiver crate.

## Boundary Rule

If a validation flow is fundamentally about persisted evidence, manifests, or
reference files, infra is a plausible owner. If it is fundamentally about
runtime stage behavior, the stronger owner is receiver.

## Protecting Proof

- `crates/bijux-gnss-infra/src/validate_reference.rs`
- `crates/bijux-gnss-infra/src/api.rs`
- `crates/bijux-gnss-infra/docs/VALIDATION.md`
