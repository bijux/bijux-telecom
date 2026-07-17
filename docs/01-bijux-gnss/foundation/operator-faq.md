---
title: Operator FAQ
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# Operator FAQ

This page keeps the old root-level FAQ inside the command owner that readers
actually start from.

## Common Routing Questions

- what time system does the command surface assume:
  route to `bijux-gnss-core` once the question stops being operator-facing and
  becomes shared time semantics
- which reference frame is used:
  route to `bijux-gnss-core` and `bijux-gnss-nav` once the question becomes
  model detail rather than top-level user expectation
- how are ionosphere and troposphere handled:
  route to `bijux-gnss-nav` once the question becomes correction-model detail
- what does deterministic mode mean:
  route to receiver and maintainer quality docs once the question becomes proof
  and regression policy

## Why This Page Is Short

The command handbook should answer reader routing honestly, not duplicate lower
technical manuals. This page exists to route recurring operator questions to the
right owner fast.
