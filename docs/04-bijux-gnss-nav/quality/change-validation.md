---
title: Change Validation
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# Change Validation

Use the smallest honest validation set that proves the changed scientific
intent.

## Minimum Validation By Change Type

- parser change:
  run the owning format family tests and one reference-backed product test when
  applicable
- orbit or time change:
  run the relevant reference tests plus one downstream estimator check if the
  state feeds solvers
- correction change:
  run the correction family tests and one dependent position, PPP, or RTK test
- estimator change:
  run the local solver family tests and the narrowest integration tests that
  prove the public behavior

## Bad Validation Patterns

- only running a broad integration test for a low-level change
- only running unit-like tests for a public solver behavior change
- accepting green tests without checking whether the affected scientific family
  was actually exercised
