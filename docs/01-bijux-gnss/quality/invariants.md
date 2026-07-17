---
title: Invariants
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# Invariants

These are the expectations that should remain true even as `bijux-gnss` grows.

## Boundary Invariants

- the command crate stays separate from lower-owner runtime, repository, and
  science internals
- public imports remain organized by command role through the binary and thin
  facade
- validation and reporting remain command-boundary presentation, not ownership
  transfer

## Public-Surface Invariants

- stable command names and flag meaning remain explicit
- workflow composition remains the command crate's owned responsibility
- the Rust facade remains thin and honest about lower ownership

## Review Invariant

If a change moves public command meaning, the docs and proof obligations must
move with it.
