---
title: Known Limitations
audience: mixed
type: explanation
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Known Limitations

`bijux-gnss-core` is deliberately strong on shared record meaning, but that
strength has limits.

## Current Limits

- it cannot prove that a downstream runtime used a contract wisely; it can only
  make the contract explicit
- artifact validation proves coherence of payload shape and value ranges, not
  full end-to-end scientific correctness
- a curated public surface still requires discipline from downstream crates not
  to reach into private modules indirectly

## Why Admit These Limits

Foundational crates become dangerous when their docs imply that strong record
contracts are equivalent to full workflow correctness.
