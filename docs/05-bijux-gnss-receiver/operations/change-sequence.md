---
title: Change Sequence
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Change Sequence

Use this sequence when a receiver-runtime change is more than a trivial edit.

## Recommended Sequence

1. identify the owning runtime family and confirm the boundary
2. inspect the relevant crate-local docs and public exports
3. make the smallest coherent code or doc change that completes one runtime
   intent
4. run the narrowest tests that prove that intent honestly
5. commit that intent before widening to the next family

## Why The Sequence Matters

This crate is broad enough that one "small" runtime or validation edit can
have public consequences. Committing by runtime intent keeps reviewable history
aligned with package meaning.

## First Proof Check

- `crates/bijux-gnss-receiver/docs/TESTS.md`
- `crates/bijux-gnss-receiver/docs/PIPELINE.md`
- `docs/05-bijux-gnss-receiver/operations/verification-commands.md`
