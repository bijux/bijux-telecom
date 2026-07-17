---
title: Invariants
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Invariants

These expectations should remain true as the maintainer binary grows.

## Boundary Invariants

- maintainer workflows stay separate from product behavior
- governed inputs remain explicitly named and documented
- maintenance evidence remains in governed repository locations

## Behavioral Invariants

- each command keeps a narrow reviewed maintainer purpose
- command failures explain the broken governance contract clearly
- benchmark comparison remains evidence-oriented rather than opaque automation
- the crate remains binary-only unless a stronger architectural reason appears

## Review Invariant

If a change moves command meaning, the matching docs and proof obligations
should move with it.
