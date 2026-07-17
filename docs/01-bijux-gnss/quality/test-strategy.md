---
title: Test Strategy
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# Test Strategy

`bijux-gnss` needs several layers of proof because it owns the public command
boundary over many deeper crates.

## Main Proof Layers

- command-shape and guardrail tests such as
  `integration_guardrails.rs`
- workflow integration tests for acquisition, navigation, validation, RINEX,
  and synthetic flows such as `integration_nav_decode.rs`,
  `integration_validate_capture.rs`, `integration_validate_config.rs`,
  `integration_validate_synthetic_navigation.rs`, `integration_rinex.rs`, and
  `integration_export_synthetic_iq.rs`
- reporting and validation publication proof through command-facing tests such
  as `integration_raw_iq_metadata.rs` and
  `integration_raw_iq_front_end_metrics.rs`
- support-helper tests in
  `src/cli/command_support/tests/` where command-owned adapters are the real
  thing being proven

## Why One Layer Is Never Enough

Passing a lower-level crate test does not prove the command boundary still
assembles the right workflow. Passing one broad integration test does not prove
that a changed flag or report path still means the same thing. The command
crate needs proof at the public workflow layer.

## Strong Example Families

- `integration_validate_config.rs`
- `integration_nav_decode.rs`
- `integration_validate_synthetic_navigation.rs`
- `integration_rinex.rs`
- `integration_validate_capture.rs`
- `src/cli/command_support/tests/`
