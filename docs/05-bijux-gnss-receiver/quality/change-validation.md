---
title: Change Validation
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Change Validation

Use the smallest honest validation set that proves the changed runtime intent.

## Minimum Validation By Change Type

- runtime-config or port change:
  run the owning runtime or boundary tests and one integration test that
  exercises the affected seam
- stage change:
  run the owning stage family tests and the narrowest integration tests that
  prove the runtime-visible effect
- artifact or validation-report change:
  run the relevant artifact, validation, or synthetic proof tests directly
- receiver-owned navigation adapter change:
  run the affected receiver tests plus one nav-backed integration proof if the
  public meaning changed

## Bad Validation Patterns

- only running a broad integration test for a stage-local change
- only running unit-like tests for a public artifact or validation behavior
- accepting green tests without checking whether the affected runtime family was
  actually exercised
