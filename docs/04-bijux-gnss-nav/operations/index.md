---
title: Operations
audience: mixed
type: index
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# Operations

Open this section when the question is how to change `bijux-gnss-nav` without
quietly moving scientific meaning, broadening public contracts carelessly, or
breaking reference-backed trust.

## Operational Model

```mermaid
flowchart LR
    change["proposed nav change"]
    scope["scope and owner check"]
    docs["scientific contract update"]
    tests["narrow verification"]
    review["cross-crate review"]
    release["merge-ready change"]

    change --> scope --> docs --> tests --> review --> release
```

## Read These First

- open [Foundation](../foundation/) first if the change may belong in another
  crate
- stay in this section when the ownership is clear and the real question is
  how to edit a scientific package safely

## First Proof Check

- `crates/bijux-gnss-nav/README.md`
- `crates/bijux-gnss-nav/docs/TESTS.md`
- `crates/bijux-gnss-nav/tests/`
