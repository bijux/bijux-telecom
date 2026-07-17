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

- command execution proof for governed-file workflows
- structure proof from `integration_guardrails`
- nextest-roster integrity proof from
  `integration_nextest_suite_selection`
- documentation and reviewed-input cross-checking for governed files and
  evidence locations

## Why Layers Matter

One passing command does not prove the repository boundary is still sound, and
one passing guardrail test does not prove a governed-file workflow still means
the same thing. The layers protect different maintainer promises.
