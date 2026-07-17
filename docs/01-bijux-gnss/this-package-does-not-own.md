---
title: This Package Does Not Own
audience: mixed
type: boundary
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# This Package Does Not Own

`bijux-gnss` is the public entrypoint, so it attracts pressure from every
neighboring crate. This ledger records the boundaries that must stay outside
the command crate unless the ownership model itself changes.

## Explicit Refusals

- receiver-stage internals and runtime-engine ownership
- repository dataset and persisted artifact ownership
- standalone navigation algorithms and precise-product interpretation
- signal catalogs, code families, and reusable DSP primitives
- generic shared contracts that do not require command-boundary meaning

## Strongest Neighboring Owners

- [03-bijux-gnss-infra](../03-bijux-gnss-infra/) for persisted repository
  mechanics, datasets, manifests, and run history
- [05-bijux-gnss-receiver](../05-bijux-gnss-receiver/) for runtime execution,
  stage orchestration, and in-memory receiver artifacts
- [04-bijux-gnss-nav](../04-bijux-gnss-nav/) for navigation science, orbit
  products, and estimator behavior
- [06-bijux-gnss-signal](../06-bijux-gnss-signal/) for signal ownership,
  sample contracts, code families, and reusable DSP
- [02-bijux-gnss-core](../02-bijux-gnss-core/) for cross-package semantic
  contracts that the CLI only consumes

## Review Trigger

Update this page when the same rejected ownership pressure keeps returning or
when the command boundary genuinely changes. Do not delete refusals just
because a command happens to call into the neighboring owner more often.
