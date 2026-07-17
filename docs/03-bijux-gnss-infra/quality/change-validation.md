---
title: Change Validation
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Change Validation

The minimum acceptable proof depends on which infra contract family changed.

## Override Or Sweep Change

- update override or experiment docs if meaning changed
- run `cargo test -p bijux-gnss-infra --test integration_overrides`

## Boundary Or Dependency Change

- confirm the stronger owner is still infra
- run `cargo test -p bijux-gnss-infra --test integration_guardrails`

## Dataset, Run-Layout, Or Validation-Adapter Change

- update the owning docs in the same change set
- inspect the relevant source family directly
- if no dedicated narrow automated proof exists yet, say that explicitly rather
  than pretending coverage exists

## Why This Honesty Matters

Infra changes can silently alter repository behavior even when product crates
still compile. Validation needs to track repository meaning, not only test
availability.
