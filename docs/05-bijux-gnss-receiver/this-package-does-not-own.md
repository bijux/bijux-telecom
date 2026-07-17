---
title: This Package Does Not Own
audience: mixed
type: boundary
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# This Package Does Not Own

`bijux-gnss-receiver` is a large runtime crate, but it still needs a refusal
ledger.

## Explicit Refusals

- command naming, CLI flags, and operator workflows
- dataset registry, repository file discovery, and persisted run-directory
  policy
- reusable signal primitives and generic DSP ownership
- standalone navigation algorithms and precise-product interpretation
- generic shared contracts that do not require receiver-runtime meaning

## Why This Page Exists

Without an explicit refusal list, readers can mistake runtime centrality for
permission to absorb every adjacent concern. That would make the handbook look
complete while making the codebase worse.

## When To Leave

- leave for [01-bijux-gnss](../01-bijux-gnss/) for command ownership
- leave for [03-bijux-gnss-infra](../03-bijux-gnss-infra/) for persisted
  repository mechanics
- leave for [04-bijux-gnss-nav](../04-bijux-gnss-nav/) for navigation science
- leave for [06-bijux-gnss-signal](../06-bijux-gnss-signal/) for signal
  ownership
