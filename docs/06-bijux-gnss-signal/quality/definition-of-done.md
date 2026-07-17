---
title: Definition Of Done
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Definition Of Done

A `bijux-gnss-signal` change is done when the public signal intent is complete,
reviewable, and honestly proven.

## Done Means

- the owning surface is explicit
- the boundary is still signal-layer, not higher-level policy
- the narrowest honest verification has been run
- any public contract drift is reflected in docs and review scope
- the commit boundary matches one durable signal intent

## First Proof Check

Inspect `crates/bijux-gnss-signal/docs/BOUNDARY.md`,
`crates/bijux-gnss-signal/docs/PUBLIC_API.md`, and
`crates/bijux-gnss-signal/docs/TESTS.md`. A change is not done until the
owning source module and matching proof family both defend the same boundary
and behavior claim.
