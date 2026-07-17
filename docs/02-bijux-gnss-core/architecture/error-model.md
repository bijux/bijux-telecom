---
title: Error Model
audience: mixed
type: explanation
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Error Model

The error model in `bijux-gnss-core` is deliberately canonical rather than
rich with runtime context.

## What The Model Owns

- stable error categories such as config, parse, input, signal, track, and nav
  failures
- diagnostic severity and summary structures
- validation failures for artifact payload coherence and config shape

## What It Does Not Own

- filesystem recovery paths
- runtime retries and backoff
- command wording or operator advice
- solver-specific remediation strategy

## Why This Matters

Core errors should be easy for downstream crates to map into their own
contexts without core pretending to know those contexts already.

## First Proof Check

- `crates/bijux-gnss-core/src/error.rs`
- `crates/bijux-gnss-core/src/diagnostic/`
- `crates/bijux-gnss-core/docs/DIAGNOSTICS.md`
