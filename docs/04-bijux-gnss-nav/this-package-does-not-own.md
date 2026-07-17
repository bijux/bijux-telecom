---
title: This Package Does Not Own
audience: mixed
type: boundary
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# This Package Does Not Own

`bijux-gnss-nav` is a large scientific crate, so it needs an explicit refusal
ledger to keep scientific breadth from turning into boundary sprawl.

## Explicit Refusals

- command naming, CLI flags, and operator flows
- dataset registry, repository file discovery, and run-directory policy
- live receiver scheduling, acquisition loops, and tracking-stage orchestration
- raw IQ generation or lower-level signal synthesis behavior
- generic shared contracts that do not require navigation meaning

## Strongest Neighboring Owners

- [01-bijux-gnss](../01-bijux-gnss/) for command ownership and operator policy
- [03-bijux-gnss-infra](../03-bijux-gnss-infra/) for persisted evidence and
  repository mechanics
- [05-bijux-gnss-receiver](../05-bijux-gnss-receiver/) for runtime stage
  control and orchestration
- [06-bijux-gnss-signal](../06-bijux-gnss-signal/) for raw-sample and
  reusable signal behavior below navigation science
- [02-bijux-gnss-core](../02-bijux-gnss-core/) for shared record meaning that
  does not require navigation ownership

## Review Trigger

Without an explicit refusal list, readers can mistake scientific breadth for
permission to absorb every adjacent concern. Update this page when repeated
review pressure reveals a new boundary worth naming.
