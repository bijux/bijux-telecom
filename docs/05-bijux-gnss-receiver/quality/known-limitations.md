---
title: Known Limitations
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Known Limitations

This handbook does not pretend the receiver crate is small or simple.

## Main Limitations

- the public API is necessarily broad because several downstream crates consume
  runtime behavior directly
- the test surface is expensive enough that choosing the right validation slice
  requires judgment
- stage families interact, so local runtime changes can feel broader than they
  first appear
- synthetic proof is powerful but still needs discipline to avoid becoming an
  unbounded truth system

## First Proof Check

- `crates/bijux-gnss-receiver/docs/TESTS.md`
- `crates/bijux-gnss-receiver/tests/prop_receiver.rs`
- `crates/bijux-gnss-receiver/tests/integration_pipeline_determinism.rs`
