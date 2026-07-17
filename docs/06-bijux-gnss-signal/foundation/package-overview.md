---
title: Package Overview
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Package Overview

`bijux-gnss-signal` is the repository's reusable signal substrate. It centralizes
the physical and computational rules that should remain stable whether the
caller is a command, a receiver pipeline, a validation surface, or a reference
test.

The crate has four durable centers of gravity:

- `catalog.rs` for signal identity, registry lookup, wavelength helpers, and
  default acquisition-signal selection
- `codes/` for constellation-specific primary and secondary code behavior
- `dsp/` for runtime-neutral signal-processing primitives
- `raw_iq.rs`, `samples.rs`, and `obs_validation.rs` for sample contracts and
  signal-layer observation checks

The crate is intentionally below receiver orchestration and above shared core
types. It depends on `bijux-gnss-core` for common meaning, but it owns the
signal-specific logic that downstream crates should reuse rather than recreate.
