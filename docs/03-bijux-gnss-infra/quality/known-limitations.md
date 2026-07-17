---
title: Known Limitations
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Known Limitations

`bijux-gnss-infra` is strong on repository contract clarity, but some proof
areas are still lighter than the surface breadth it owns.

## Current Limits

- dedicated automated tests are narrower than the full dataset, run-layout, and
  validation-adapter surface
- infra can prove repository interpretation discipline, not the end-to-end
  scientific correctness of product outputs
- selected lower-level re-exports in `api.rs` require continued discipline so
  callers do not mistake convenience for infra ownership

## Why Admit These Limits

Repository-facing crates become dangerous when they imply that typed file and
artifact handling is equivalent to complete workflow correctness.
