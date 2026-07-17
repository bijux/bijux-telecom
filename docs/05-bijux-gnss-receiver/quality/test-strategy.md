---
title: Test Strategy
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Test Strategy

`bijux-gnss-receiver` needs several layers of proof because it owns both
runtime orchestration and a large stage-facing integration surface.

## Main Proof Layers

- stage-family tests for acquisition, tracking, and observations such as
  `integration_acquisition_*.rs`, `integration_tracking_*.rs`, and
  `integration_observations_*.rs`
- runtime integration tests for boundary, determinism, and support reporting
  such as `integration_basic.rs`, `integration_determinism.rs`,
  `integration_pipeline_determinism.rs`, and
  `integration_receiver_support_matrix_inventory.rs`
- validation and synthetic tests for runtime proof behavior such as
  `integration_navigation_validation_run.rs`,
  `integration_synthetic.rs`, `integration_tracking_truth_table.rs`, and
  `integration_navigation_pvt_truth_table.rs`
- feature-gated navigation and RTK tests when receiver-owned adapters or
  validation surfaces depend on nav, such as `integration_navigation_*.rs` and
  `integration_rtk_*.rs`
- golden and truth-table tests for stable runtime outputs such as
  `golden_acquisition.rs`, `golden_tracking.rs`, and the truth-table suites

## Why One Layer Is Never Enough

Passing a broad receiver integration test does not prove a tracking-loop change
is safe. Passing a stage-local test does not prove runtime artifact meaning or
validation behavior still holds. The crate needs proof at the owning runtime
layer and at the most relevant integration layer.

## Strong Example Families

- `integration_basic.rs`
- `integration_receiver_support_matrix_inventory.rs`
- `integration_acquisition_*.rs`
- `integration_tracking_*.rs`
- `integration_observations_*.rs`
- `integration_navigation_*.rs`
- `integration_rtk_*.rs`
