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

- `src/catalog.rs` for signal identity, registry lookup, wavelength helpers,
  and default acquisition-signal selection
- `src/codes/` for constellation-specific primary and secondary code behavior
- `src/dsp/` for runtime-neutral signal-processing primitives
- `src/raw_iq.rs`, `src/samples.rs`, and `src/obs_validation.rs` for sample
  contracts and signal-layer observation checks

The crate is intentionally below receiver orchestration and above shared core
types. It depends on `bijux-gnss-core` for common meaning, but it owns the
signal-specific logic that downstream crates should reuse rather than recreate.

The public surface is deliberately wider than a single algorithm family but
still narrow in responsibility: catalog truth, code-generation truth, reusable
DSP, sample contracts, and signal-layer validation.

## First Proof Check

- `crates/bijux-gnss-signal/src/api.rs`
- `crates/bijux-gnss-signal/src/catalog.rs`
- `crates/bijux-gnss-signal/src/codes/mod.rs`
- `crates/bijux-gnss-signal/src/dsp/mod.rs`
- `crates/bijux-gnss-signal/src/obs_validation.rs`
