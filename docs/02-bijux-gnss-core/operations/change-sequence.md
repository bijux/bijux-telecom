---
title: Change Sequence
audience: mixed
type: explanation
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Change Sequence

Use this sequence for any real `bijux-gnss-core` change.

1. decide whether the change truly belongs in core
2. identify the contract family and public-surface impact
3. update the owning docs in `crates/bijux-gnss-core/docs/` if the meaning
   changes
4. update the root handbook pages here if package-level reader guidance changes
5. run the narrowest protecting tests for that family
6. only then look for downstream fallout in higher-level crates

Skipping the first step is the most expensive mistake. A clean downstream owner
is often cheaper than a permanent new core contract.
