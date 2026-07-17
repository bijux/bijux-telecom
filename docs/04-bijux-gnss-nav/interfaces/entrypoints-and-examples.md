---
title: Entrypoints And Examples
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# Entrypoints And Examples

Use this page to choose the right public starting point.

## Common Starting Points

- start from `bijux_gnss_nav::api` when you are a downstream crate
- start from the relevant format family when the question begins with external
  navigation data
- start from `PositionSolver`, `NavigationEngine`, or `PositionRuntime` when
  the question is position behavior
- start from `PppFilter` when the question is precise point positioning state
  and lifecycle
- start from RTK execution, ambiguity, or quality helpers when the question is
  differencing and baseline estimation

## Example Reader Routes

- "I need to parse an SP3 and feed it to PPP":
  precise-product contracts, then PPP product-policy surfaces
- "I need to understand why a position solve was refused":
  estimation contracts, then position solution and integrity evidence
- "I need broadcast time interpretation for a constellation-specific decoder":
  format contracts, then time contracts
