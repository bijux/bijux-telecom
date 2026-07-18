---
title: This Package Does Not Own
audience: mixed
type: boundary
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# This Package Does Not Own

`bijux-gnss-infra` is repository-facing infrastructure over product behavior.
This ledger keeps file layout, dataset state, and persisted evidence from
becoming an excuse to absorb neighboring product ownership.

## Explicit Refusals

- receiver stage orchestration, tracking sessions, or runtime ports
- signal processing, sample math, or DSP primitives
- orbit parsing, corrections, or navigation estimators
- operator command parsing, report rendering, or command vocabulary
- shared semantic record meaning that belongs in `bijux-gnss-core`

## Strongest Neighboring Owners

- [Receiver handbook](../05-bijux-gnss-receiver/) for runtime execution
  and in-memory artifacts before persistence
- [Signal handbook](../06-bijux-gnss-signal/) for signal behavior and
  raw-sample meaning
- [Navigation handbook](../04-bijux-gnss-nav/) for navigation science and
  reference-product interpretation
- [Command handbook](../01-bijux-gnss/) for command-line policy and report shape
- [Core handbook](../02-bijux-gnss-core/) for cross-package semantic
  records and envelopes

## Review Trigger

If the proposal sounds like product behavior with a file nearby, infra is
probably the wrong owner. Update this ledger when the same boundary mistake
shows up repeatedly in review.

## First Neighbor Proof Check

- [Receiver handbook](../05-bijux-gnss-receiver/index.md)
- [Signal handbook](../06-bijux-gnss-signal/index.md)
- [Navigation handbook](../04-bijux-gnss-nav/index.md)
- [Command handbook](../01-bijux-gnss/index.md)
- [Core handbook](../02-bijux-gnss-core/index.md)
