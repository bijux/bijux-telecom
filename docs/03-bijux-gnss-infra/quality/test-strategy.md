---
title: Test Strategy
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Test Strategy

The test strategy in `bijux-gnss-infra` is currently strongest around typed
override behavior and boundary guardrails, with the rest of the trust story
carried partly by documentation and explicit module ownership.

## Main Test Families

- override application tests in `tests/integration_overrides.rs`
- workspace guardrail and boundary tests in
  `tests/integration_guardrails.rs`

## Strategy Rule

Tests here should prove infrastructure ownership and repository-state behavior.
They should not duplicate runtime, signal, or navigation behavior from stronger
product owners.

## Current Truth

The crate has narrower automated coverage than the full breadth of its
repository contract surface. That is not something to hide. It means dataset,
run-layout, and validation-adapter changes need disciplined documentation and
review, not just a green narrow test. The strongest current proof is therefore
mixed: narrow automated tests plus explicit checked-in contract docs.

## Protecting Proof

- `crates/bijux-gnss-infra/tests/integration_overrides.rs`
- `crates/bijux-gnss-infra/tests/integration_guardrails.rs`
- `crates/bijux-gnss-infra/docs/TESTS.md`
