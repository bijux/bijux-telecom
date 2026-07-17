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

## Strongest Neighboring Owners

- [01-bijux-gnss](../01-bijux-gnss/) for command ownership and operator
  workflow policy
- [03-bijux-gnss-infra](../03-bijux-gnss-infra/) for persisted repository
  mechanics and run layout
- [04-bijux-gnss-nav](../04-bijux-gnss-nav/) for navigation science and
  estimator behavior
- [06-bijux-gnss-signal](../06-bijux-gnss-signal/) for signal ownership,
  sample contracts, and reusable DSP
- [02-bijux-gnss-core](../02-bijux-gnss-core/) for shared exchange records and
  artifact envelopes

## Review Trigger

Without an explicit refusal list, readers can mistake runtime centrality for
permission to absorb every adjacent concern. Update this page when new review
pressure keeps trying to move neighboring ownership into the runtime crate.

## First Neighbor Proof Check

Inspect [01-bijux-gnss](../01-bijux-gnss/),
[03-bijux-gnss-infra](../03-bijux-gnss-infra/),
[04-bijux-gnss-nav](../04-bijux-gnss-nav/),
[06-bijux-gnss-signal](../06-bijux-gnss-signal/), and
[02-bijux-gnss-core](../02-bijux-gnss-core/) before widening receiver
ownership. That check is what keeps runtime centrality from hardening into
dishonest cross-owner sprawl.
