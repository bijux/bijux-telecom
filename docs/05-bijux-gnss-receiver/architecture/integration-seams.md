---
title: Integration Seams
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Integration Seams

This page names the places where `bijux-gnss-receiver` intentionally meets
neighboring crates without surrendering ownership.

## Public API Seam

`src/api.rs` is the deliberate integration seam for downstream crates. If a
consumer needs internal module paths instead of `api.rs`, the receiver boundary
may be too leaky.

## Port Seams

- `SampleSource` and `ArtifactSink` in `src/ports/mod.rs`
- `Clock` and `SystemClock` in `src/ports/clock.rs`
- `FileSamples` and `MemorySamples` adapters in `src/io/data.rs`

These seams let the runtime interact with input, output, and time without
hardwiring one repository or command environment.

## Stage And Validation Seams

- `ReceiverEngine` and `Receiver` in `src/engine/engine.rs` and
  `src/engine/receiver.rs` as top-level runtime seams
- `Navigation` and `NavigationFilter` as receiver-owned adapters over
  nav-owned science through `src/pipeline/navigation.rs` and
  `src/pipeline/navigation_filter.rs`
- runtime-side validation and synthetic helpers that expose receiver behavior
  through `src/reference_validation.rs`, `src/validation_report.rs`, and
  `src/sim/synthetic/` without turning the crate into a repository tool

## Boundary Rule

Integration seams should pass runtime inputs and outputs. They should not
smuggle repository persistence, command policy, or reimplemented signal or nav
science into the crate.
