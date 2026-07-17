---
title: Verification Commands
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Verification Commands

Use the narrowest relevant command that still proves the changed runtime
surface.

## Focused Commands From Repository Root

```sh
cargo test -p bijux-gnss-receiver --test integration_basic
cargo test -p bijux-gnss-receiver --test integration_receiver_support_matrix_inventory
cargo test -p bijux-gnss-receiver --test integration_navigation_pvt_accuracy_budget
```

## Additional Targeted Checks Worth Using

```sh
cargo test -p bijux-gnss-receiver --test integration_acquisition_smoke
cargo test -p bijux-gnss-receiver --test integration_tracking_cn0
cargo test -p bijux-gnss-receiver --test integration_observations_measurement_quality
```

## Command Choice Rule

- use acquisition tests for acquisition changes
- use tracking tests for loop, channel, or reacquisition changes
- use observation tests before wider navigation tests when possible
- use validation or synthetic tests directly when runtime proof logic changes
