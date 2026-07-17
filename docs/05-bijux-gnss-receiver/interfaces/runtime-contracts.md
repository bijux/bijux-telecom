---
title: Runtime Contracts
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Runtime Contracts

These contracts define the top-level receiver boundary.

## Owned Runtime Surfaces

- `ReceiverConfig` and `ReceiverPipelineConfig` in
  `src/engine/receiver_config.rs` and the surrounding config family
- `ReceiverRuntimeConfig` and `ReceiverRuntime` in `src/engine/runtime.rs`
- logger, trace, and metrics sink traits
- `Receiver`, `ReceiverEngine`, and receiver-launch helpers in
  `src/engine/receiver.rs` and `src/engine/engine.rs`
- support-matrix and diagnostics-facing runtime output support

## Stability Expectations

- callers may rely on receiver configuration and runtime controls being owned
  here
- runtime contracts should remain free of command-only or repository-only
  policy
- top-level entrypoints should expose receiver behavior without forcing callers
  into stage internals

## Closest Proof

- `crates/bijux-gnss-receiver/src/engine/`
- `crates/bijux-gnss-receiver/tests/integration_basic.rs`
- `crates/bijux-gnss-receiver/tests/integration_receiver_support_matrix_inventory.rs`
- `crates/bijux-gnss-receiver/docs/RUNTIME.md`
