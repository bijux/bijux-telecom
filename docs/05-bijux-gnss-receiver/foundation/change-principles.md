---
title: Change Principles
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Change Principles

Changes to `bijux-gnss-receiver` should preserve runtime legibility, not only
make the tests pass.

## Principles

- keep runtime composition near the families that own it instead of building a
  giant convenience layer
- let signal and navigation crates own their science even when the runtime
  consumes it heavily
- treat artifacts, traces, and validation helpers as receiver-boundary
  contracts, not as persistence policy
- widen the public receiver API only when multiple downstream owners genuinely
  need a stable runtime contract
- keep synthetic helpers honest about exercising receiver behavior rather than
  replacing lower-level truth ownership

## Warning Signs

- a new helper is easier to describe by one caller than by its runtime role
- receiver configuration starts embedding command defaults or report wording
- runtime artifacts start depending on repository path or manifest assumptions
