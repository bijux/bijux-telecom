---
title: Test Strategy
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Test Strategy

`bijux-gnss-dev` needs layered proof because it governs repository behavior
rather than one isolated algorithm.

## Main Proof Layers

- command execution proof for `audit-allowlist`, `deny-policy-deviations`, and
  `audit-ignore-args`
- structure proof from `integration_guardrails`
- nextest-roster integrity proof from
  `integration_nextest_suite_selection`
- documentation and reviewed-input cross-checking for governed files and
  evidence locations
- benchmark comparison proof through `bench-compare`, with explicit allowance
  to report when full benchmark execution is too expensive for the current pass

## Why Layers Matter

One passing command does not prove the repository boundary is still sound, and
one passing guardrail test does not prove a governed-file workflow still means
the same thing. The layers protect different maintainer promises.

## Protecting Proof

- `crates/bijux-gnss-dev/docs/TESTS.md`
- `crates/bijux-gnss-dev/tests/integration_guardrails.rs`
- `crates/bijux-gnss-dev/tests/integration_nextest_suite_selection.rs`
- `crates/bijux-gnss-dev/docs/BENCHMARKS.md`
