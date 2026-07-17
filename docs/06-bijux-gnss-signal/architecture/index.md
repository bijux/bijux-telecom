---
title: Architecture
audience: mixed
type: index
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Architecture

Open this section when the question is where signal behavior lives in code and
why the source tree is partitioned the way it is.

## What This Section Covers

- the module map from `catalog.rs` through `codes/`, `dsp/`, and contract files
- dependency direction inside the crate
- the runtime-neutral execution model for reusable signal math
- integration seams between code families, DSP primitives, and validation

## First Code Roots

- `crates/bijux-gnss-signal/src/catalog.rs`
- `crates/bijux-gnss-signal/src/codes/`
- `crates/bijux-gnss-signal/src/dsp/`
- `crates/bijux-gnss-signal/src/raw_iq.rs`
- `crates/bijux-gnss-signal/src/samples.rs`
- `crates/bijux-gnss-signal/src/obs_validation.rs`
