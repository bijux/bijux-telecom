---
title: This Package Does Not Own
audience: mixed
type: boundary
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# This Package Does Not Own

`bijux-gnss-nav` is a large crate, but it still needs a refusal ledger.

## Explicit Refusals

- command naming, CLI flags, and operator flows
- dataset registry, repository file discovery, and run-directory policy
- live receiver scheduling, acquisition loops, and tracking-stage orchestration
- raw IQ generation or lower-level signal synthesis behavior
- generic shared contracts that do not require navigation meaning

## Why This Page Exists

Without an explicit refusal list, readers can mistake the crate's scientific
breadth for permission to absorb every adjacent concern. That would make the
handbook clearer while making the codebase worse.

## When To Leave

- leave for [01-bijux-gnss](../01-bijux-gnss/) for command ownership
- leave for [03-bijux-gnss-infra](../03-bijux-gnss-infra/) for persisted
  evidence and repository mechanics
- leave for [05-bijux-gnss-receiver](../05-bijux-gnss-receiver/) for runtime
  stage control
