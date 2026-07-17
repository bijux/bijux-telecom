---
title: This Package Does Not Own
audience: mixed
type: boundary
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# This Package Does Not Own

`bijux-gnss` is the public entrypoint, but it still needs a refusal ledger.

## Explicit Refusals

- receiver-stage internals and runtime-engine ownership
- repository dataset and persisted artifact ownership
- standalone navigation algorithms and precise-product interpretation
- signal catalogs, code families, and reusable DSP primitives
- generic shared contracts that do not require command-boundary meaning

## Why This Page Exists

Without an explicit refusal list, readers can mistake public entrypoint status
for permission to absorb every adjacent concern. That would make the handbook
look complete while making the codebase worse.

## When To Leave

- leave for [03-bijux-gnss-infra](../03-bijux-gnss-infra/) for persisted
  repository mechanics
- leave for [05-bijux-gnss-receiver](../05-bijux-gnss-receiver/) for runtime
  execution
- leave for [04-bijux-gnss-nav](../04-bijux-gnss-nav/) for navigation science
- leave for [06-bijux-gnss-signal](../06-bijux-gnss-signal/) for signal
  ownership
