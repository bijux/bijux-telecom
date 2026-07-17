---
title: Test Strategy
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Test Strategy

The test strategy in `bijux-gnss-core` is narrow by family and high in
amplification.

## Main Test Families

- public-surface guardrail tests in `tests/public_api_guardrail.rs`
- artifact payload validation tests in `tests/nav_artifact_validation.rs` and
  `tests/tracking_artifact_validation.rs`
- property tests for timekeeping and conversion behavior in
  `tests/prop_timekeeping.rs`
- workspace guardrails for dependency and layering posture in
  `tests/integration_guardrails.rs`

## Strategy Rule

Tests here should prove contract stability, not duplicate runtime behavior from
higher-level crates. If a test reads like receiver orchestration or navigation
policy, the stronger owner is probably elsewhere.

## Protecting Proof

- `crates/bijux-gnss-core/tests/public_api_guardrail.rs`
- `crates/bijux-gnss-core/tests/nav_artifact_validation.rs`
- `crates/bijux-gnss-core/tests/tracking_artifact_validation.rs`
- `crates/bijux-gnss-core/tests/prop_timekeeping.rs`
- `crates/bijux-gnss-core/tests/prop_timekeeping.proptest-regressions`
- `crates/bijux-gnss-core/tests/integration_guardrails.rs`
