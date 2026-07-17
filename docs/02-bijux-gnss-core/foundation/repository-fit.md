---
title: Repository Fit
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Repository Fit

`bijux-gnss-core` sits near the bottom of the GNSS package stack. It should
feel easy to depend on and hard to misuse.

## Fit In The Repository

- `bijux-gnss-signal` depends on core meaning to describe signals, samples, and
  validation outputs
- `bijux-gnss-nav` depends on core records to represent observations,
  solutions, and diagnostics
- `bijux-gnss-receiver` depends on core contracts to exchange stage state and
  emit runtime artifacts
- `bijux-gnss-infra` depends on core artifacts and validation reports when it
  inspects or persists outputs
- `bijux-gnss` depends on the same language indirectly when it exposes top-level
  workflows

## Why This Fit Matters

If core grows heavy dependencies or local policy, every consumer pays the
price. Foundational crates should lower coordination cost, not raise it.

## Repository Smell

When a higher-level crate can no longer explain its own behavior without also
importing core implementation detail, the package seam has already weakened.

## First Proof Check

- repository `Cargo.toml`
- `crates/bijux-gnss-core/src/lib.rs`
- `crates/bijux-gnss-core/tests/integration_guardrails.rs`
