---
title: Change Validation
audience: mixed
type: explanation
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Change Validation

The minimum acceptable proof depends on which contract family changed.

## Public Export Change

- update `api.rs` deliberately
- update crate-local public API docs
- run `cargo test -p bijux-gnss-core --test public_api_guardrail`

## Artifact Or Validation Change

- update serialization or contract prose
- run artifact validation tests
- update any affected fixture with an explicit semantic reason

## Timekeeping Change

- update invariant language if behavior changed
- run `cargo test -p bijux-gnss-core --test prop_timekeeping`
- inspect whether the regression corpus changed for a good reason

## Boundary Change

- verify the stronger owner is still core
- run `cargo test -p bijux-gnss-core --test integration_guardrails`
