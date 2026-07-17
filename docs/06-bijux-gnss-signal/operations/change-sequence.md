---
title: Change Sequence
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Change Sequence

Use this sequence when modifying `bijux-gnss-signal`.

1. identify the owning signal surface:
   catalog, code family, DSP primitive, sample contract, or validation rule
2. read the adjacent crate-local docs and matching root-handbook section
3. inspect the current proof family in `tests/` before editing behavior
4. change the owning module and only the closely coupled docs or tests
5. run the narrowest honest verification commands for that owner
6. review whether `src/api.rs` or contract docs must change as part of the same
   intent

## Why This Order Matters

The wrong order usually causes one of two failures: a supposedly local change
quietly breaks a downstream contract, or a broad test passes while the relevant
signal owner was never actually exercised.

## First Proof Check

Read `crates/bijux-gnss-signal/docs/TESTS.md`,
`crates/bijux-gnss-signal/docs/PUBLIC_API.md`, and
`crates/bijux-gnss-signal/docs/BOUNDARY.md` before editing. Then inspect the
owning module under `crates/bijux-gnss-signal/src/` and the matching proof
family under `crates/bijux-gnss-signal/tests/` so the change sequence starts
from actual contracts instead of from convenient local edits.
