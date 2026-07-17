---
title: This Package Does Not Own
audience: mixed
type: boundary
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# This Package Does Not Own

This page is the refusal ledger for `bijux-gnss-signal`. When a change feels
technically close to signal math but still belongs elsewhere, record the
decision here instead of letting the boundary blur.

## Refusals

- receiver stage sequencing, channel scheduling, and run-budget policy belong
  in `bijux-gnss-receiver`
- persisted capture lookup, dataset layout, and repository-side metadata belong
  in `bijux-gnss-infra`
- navigation estimator confidence, orbit-domain state, and solution acceptance
  belong in `bijux-gnss-nav`
- operator commands, workflow sequencing, and report-facing policy belong in
  `bijux-gnss`
- cross-package IDs, units, and generic observation record meaning belong in
  `bijux-gnss-core`

## Strongest Neighboring Owners

- [05-bijux-gnss-receiver](../05-bijux-gnss-receiver/) for runtime sequencing,
  channels, and run-budget policy
- [03-bijux-gnss-infra](../03-bijux-gnss-infra/) for persisted capture lookup,
  dataset layout, and repository metadata
- [04-bijux-gnss-nav](../04-bijux-gnss-nav/) for navigation judgment and
  solution acceptance
- [01-bijux-gnss](../01-bijux-gnss/) for commands and operator workflow policy
- [02-bijux-gnss-core](../02-bijux-gnss-core/) for shared IDs, units, and
  cross-package observation meaning

## Review Trigger

- add an entry when a rejected design pressure keeps returning
- link the owning crate in review discussions
- prefer a clear refusal over a convenience abstraction that nobody will own
  later
- if the strongest sentence being defended is about solution truth, runtime
  acceptability, dataset persistence, or command policy, the signal crate is
  the wrong owner even when signal math appears in the implementation
