---
title: Definition of Done
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Definition of Done

A `bijux-gnss-core` change is not done when code compiles. It is done when the
contract change, if any, is legible and defended.

## Done Means

- ownership is still correct
- affected contract docs are aligned
- the narrow protecting tests are green
- fixtures or regressions changed only with explicit reason
- no higher-level dependency or policy drift was introduced

## Not Done Yet Means

- a helper was made public "for now"
- serialized meaning changed but the docs still describe the old behavior
- invariants are implicit in test names only
- the crate absorbed behavior because it was easier than naming a downstream
  owner
