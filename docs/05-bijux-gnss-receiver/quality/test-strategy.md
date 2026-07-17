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

- stage-family tests for acquisition, tracking, and observations
- runtime integration tests for boundary, determinism, and support reporting
- validation and synthetic tests for runtime proof behavior
- feature-gated navigation and RTK tests when receiver-owned adapters or
  validation surfaces depend on nav
- golden and truth-table tests for stable runtime outputs

## Why One Layer Is Never Enough

Passing a broad receiver integration test does not prove a tracking-loop change
is safe. Passing a stage-local test does not prove runtime artifact meaning or
validation behavior still holds. The crate needs proof at the owning runtime
layer and at the most relevant integration layer.

## Strong Example Families

- `integration_basic`
- `integration_receiver_support_matrix_inventory`
- `integration_acquisition_*`
- `integration_tracking_*`
- `integration_observations_*`
- `integration_navigation_*`
