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

- [05-bijux-gnss-receiver](../05-bijux-gnss-receiver/) for runtime execution
  and in-memory artifacts before persistence
- [06-bijux-gnss-signal](../06-bijux-gnss-signal/) for signal behavior and
  raw-sample meaning
- [04-bijux-gnss-nav](../04-bijux-gnss-nav/) for navigation science and
  reference-product interpretation
- [01-bijux-gnss](../01-bijux-gnss/) for command-line policy and report shape
- [02-bijux-gnss-core](../02-bijux-gnss-core/) for cross-package semantic
  records and envelopes

## Review Trigger

If the proposal sounds like product behavior with a file nearby, infra is
probably the wrong owner. Update this ledger when the same boundary mistake
shows up repeatedly in review.

## First Neighbor Proof Check

- `../05-bijux-gnss-receiver/index.md`
- `../06-bijux-gnss-signal/index.md`
- `../04-bijux-gnss-nav/index.md`
- `../01-bijux-gnss/index.md`
- `../02-bijux-gnss-core/index.md`
