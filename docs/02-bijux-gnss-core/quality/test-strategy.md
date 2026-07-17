---
title: Test Strategy
audience: mixed
type: explanation
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Test Strategy

The test strategy in `bijux-gnss-core` is narrow by family and high in
amplification.

## Main Test Families

- public-surface guardrail tests
- artifact payload validation tests
- property tests for timekeeping and conversion behavior
- workspace guardrails for dependency and layering posture

## Strategy Rule

Tests here should prove contract stability, not duplicate runtime behavior from
higher-level crates. If a test reads like receiver orchestration or navigation
policy, the stronger owner is probably elsewhere.

## Protecting Proof

- `crates/bijux-gnss-core/tests/public_api_guardrail.rs`
- `crates/bijux-gnss-core/tests/nav_artifact_validation.rs`
- `crates/bijux-gnss-core/tests/tracking_artifact_validation.rs`
- `crates/bijux-gnss-core/tests/prop_timekeeping.rs`
- `crates/bijux-gnss-core/tests/integration_guardrails.rs`
