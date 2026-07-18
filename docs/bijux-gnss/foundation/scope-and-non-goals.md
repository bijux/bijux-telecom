---
title: Scope And Non-Goals
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# Scope And Non-Goals

This page states what `bijux-gnss` is allowed to own and what it should refuse
even when command workflows depend heavily on neighboring crates.

## In Scope

- the `bijux` binary and stable command vocabulary
- argument parsing and command dispatch
- top-level workflow wiring across lower-level GNSS crates
- operator-facing reporting and output shape
- the thin package facade over the lower-level Rust crates

## Explicit Non-Goals

- receiver-runtime internals and stage execution policy
- repository run layout, manifests, and persisted artifact inspection
- standalone navigation algorithms or precise-product interpretation
- signal primitives, code families, or reusable DSP ownership
- generic shared contracts without command-boundary meaning

## Why These Refusals Matter

If the command crate absorbs runtime, persistence, or science ownership,
operators lose a clean public entrypoint and reviewers lose the ability to
challenge claims at the right boundary.

## Strong Negative Test

If a change cannot be explained in terms of command vocabulary, top-level
workflow composition, output shape, or facade convenience, it probably does not
belong here.
