---
title: Local Development
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Local Development

Local development in `bijux-gnss-core` should stay narrow and proof-oriented.

## Good Development Loop

1. identify the contract family first
2. inspect the owning source area and crate-local docs
3. make the smallest coherent contract change
4. run the narrowest protecting tests
5. only then expand into broader workspace fallout if needed

## Why Narrow First

Because this crate sits underneath many others, broad workspace runs are often
less informative than a well-chosen narrow check at the contract boundary.

## First Proof Check

- `crates/bijux-gnss-core/src/`
- `crates/bijux-gnss-core/docs/`
- `crates/bijux-gnss-core/tests/`
