---
title: Definition Of Done
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Definition Of Done

A `bijux-gnss-dev` change is done when the maintainer workflow intent is
complete, reviewable, and honestly proven.

## Done Means

- the owning workflow is explicit
- the boundary is still maintainer-only, not product-facing
- the narrowest honest verification has been run
- governed inputs and outputs still match the docs
- the commit boundary matches one durable maintainer intent

## First Proof Check

Inspect `crates/bijux-gnss-dev/docs/BOUNDARY.md`,
`crates/bijux-gnss-dev/docs/WORKFLOWS.md`,
`crates/bijux-gnss-dev/docs/OUTPUTS.md`, and
`crates/bijux-gnss-dev/src/main.rs`. A change is not done until the command
path and its governed input or output contract still defend the same reader
promise.
