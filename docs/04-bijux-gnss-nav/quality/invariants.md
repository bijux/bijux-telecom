---
title: Invariants
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# Invariants

These are the expectations that should remain true even as the crate grows.

## Boundary Invariants

- navigation science stays separate from runtime orchestration
- repository layout and file discovery do not leak into parsers or solvers
- public imports remain organized by scientific role through `api.rs`

## Scientific Invariants

- external navigation truth is represented by typed records or typed rejection
  evidence, not by ad hoc caller interpretation
- refusals, integrity results, and downgrade evidence are first-class outcomes
- time interpretation remains explicit when constellation-specific rules are in
  play
- PPP and RTK state machines keep their evidence and policy surfaces reviewable

## Review Invariant

If a change moves public scientific meaning, the docs and proof obligations
must move with it.

## First Proof Check

Inspect `crates/bijux-gnss-nav/docs/PUBLIC_API.md`,
`crates/bijux-gnss-nav/docs/ESTIMATION.md`,
`crates/bijux-gnss-nav/docs/TESTS.md`, and the most relevant reference or
estimator tests to confirm the invariants named here still have checked
scientific proof.
