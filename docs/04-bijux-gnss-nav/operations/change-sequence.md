---
title: Change Sequence
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# Change Sequence

Use this sequence when a navigation-science change is more than a trivial edit.

## Recommended Sequence

1. identify the owning scientific family and confirm the boundary
2. inspect the relevant crate-local docs and public exports
3. make the smallest coherent code or doc change that completes one scientific
   intent
4. run the narrowest tests that prove that intent honestly
5. commit that intent before widening to the next family

## Why The Sequence Matters

This crate is broad enough that one "small" solver or parser edit can have
public consequences. Committing by scientific intent keeps reviewable history
aligned with package meaning.

## First Proof Check

- `crates/bijux-gnss-nav/docs/TESTS.md`
- `crates/bijux-gnss-nav/docs/ESTIMATION.md`
- `docs/04-bijux-gnss-nav/operations/verification-commands.md`
