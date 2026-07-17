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

## How To Use This Ledger

- add an entry when a rejected design pressure keeps returning
- link the owning crate in review discussions
- prefer a clear refusal over a convenience abstraction that nobody will own
  later
