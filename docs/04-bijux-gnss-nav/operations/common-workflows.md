---
title: Common Workflows
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# Common Workflows

This page describes the recurring edit patterns in `bijux-gnss-nav`.

## Add Or Change A Format Family

- confirm the change belongs in a domain decoder, not in repository I/O
- update or add the relevant format documentation under
  `crates/bijux-gnss-nav/docs/`
- run the narrowest constellation or product-specific tests first
- check whether the public API changed or only the internal decoder behavior

## Change Orbit Or Time Interpretation

- verify whether the change affects only one constellation or a shared rule
- check downstream estimator assumptions before widening the change
- run reference-backed orbit and time tests, not only parser tests

## Change Correction Law

- confirm the change is reusable scientific behavior rather than one caller
  workaround
- run the most specific correction and position tests that exercise the change
- review whether new evidence types or thresholds alter public meaning

## Change Estimator Behavior

- identify whether the change is local to position, PPP, or RTK
- run the most local solver tests first, then the smallest integration tests
  that prove the public effect
- review refusal, downgrade, and integrity evidence as carefully as success
  outputs
