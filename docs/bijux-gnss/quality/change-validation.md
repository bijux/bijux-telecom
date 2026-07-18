---
title: Change Validation
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# Change Validation

Use the smallest honest validation set that proves the changed command intent.

## Minimum Validation By Change Type

- command-shape change:
  run the relevant command or config tests plus one integration test that
  exercises the affected workflow
- workflow-wiring change:
  run the narrowest integration tests that prove the intended lower-owner
  composition
- reporting or validation-output change:
  run the workflow tests that reach the changed output path
- facade change:
  confirm the export belongs here and run the smallest relevant package-facing
  proof

## Bad Validation Patterns

- only running lower-level crate tests for a command-boundary change
- only running one broad integration test for a public flag or reporting change
- accepting green tests without checking whether the affected command family was
  actually exercised
