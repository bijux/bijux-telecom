---
title: Change Sequence
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# Change Sequence

Use this sequence when a command-boundary change is more than a trivial edit.

## Recommended Sequence

1. identify the owning command family and confirm the boundary
2. inspect the relevant crate-local docs and public surfaces
3. make the smallest coherent code or doc change that completes one command
   intent
4. run the narrowest tests that prove that intent honestly
5. commit that intent before widening to the next family

## Why The Sequence Matters

This crate sits at the top of the stack, so one "small" command change can
quietly shift workflow meaning across several lower crates. Committing by
command intent keeps history aligned with the public surface.
