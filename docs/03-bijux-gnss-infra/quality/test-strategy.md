---
title: Test Strategy
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Test Strategy

The test strategy in `bijux-gnss-infra` is strongest around typed override
behavior and boundary guardrails, but it also includes focused module tests for
dataset parsing, raw-IQ metadata resolution, provenance hashing, artifact
inspection, and run-layout provenance helpers.

## Main Test Families

- override application tests in `tests/integration_overrides.rs`
- workspace guardrail and boundary tests in
  `tests/integration_guardrails.rs`
- dataset registry and raw-IQ metadata module tests in
  `src/datasets/registry.rs` and `src/datasets/raw_iq_metadata.rs`
- coordinate parsing tests in `src/parse/coordinates.rs`
- artifact inspection tests in `src/artifact_inspection/tests.rs`
- provenance hashing and front-end provenance tests in
  `src/hash/provenance.rs` and `src/run_layout/provenance/front_end.rs`
- override parameter module tests in `src/overrides/receiver_profile.rs` and
  `src/overrides/sweep_parameters.rs`

## Strategy Rule

Tests here should prove infrastructure ownership and repository-state behavior.
They should not duplicate runtime, signal, or navigation behavior from stronger
product owners.

## Current Truth

Automated coverage is real but uneven. Dataset parsing, override mechanics,
artifact inspection, and provenance helpers have direct tests today. Run-layout
persistence semantics and validation-adapter composition still depend more
heavily on checked-in contract docs and reviewer discipline than on dedicated
integration coverage. The honest proof story is therefore mixed: meaningful
automated tests plus explicit contract surfaces for the gaps.

## Protecting Proof

- `crates/bijux-gnss-infra/tests/integration_overrides.rs`
- `crates/bijux-gnss-infra/tests/integration_guardrails.rs`
- `crates/bijux-gnss-infra/docs/TESTS.md`
