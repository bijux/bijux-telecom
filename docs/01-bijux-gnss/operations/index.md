---
title: Operations
audience: mixed
type: index
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# Operations

Open this section when the question is how to change `bijux-gnss` without
quietly moving command meaning, widening public contracts carelessly, or
breaking top-level workflow composition.

## Operational Model

```mermaid
flowchart LR
    change["proposed command change"]
    scope["scope and owner check"]
    docs["command contract update"]
    tests["narrow verification"]
    review["cross-crate review"]
    release["merge-ready change"]

    change --> scope --> docs --> tests --> review --> release
```

## Read These First

- open [Change Sequence](change-sequence.md) first when the work touches
  command shape, workflow wiring, reporting, or facade exports
- open [Verification Commands](verification-commands.md) when you need the
  narrowest executable proof for a local command change
- open [Review Scope](review-scope.md) when a change seems to affect more than
  one command family at once

## Pages In This Section

- [Common Workflows](common-workflows.md)
- [Local Development](local-development.md)
- [Change Sequence](change-sequence.md)
- [Verification Commands](verification-commands.md)
- [Fixture And Workflow Care](fixture-and-workflow-care.md)
- [Review Scope](review-scope.md)
- [Release And Versioning](release-and-versioning.md)

## First Proof Check

- `crates/bijux-gnss/README.md`
- `crates/bijux-gnss/docs/TESTS.md`
- `crates/bijux-gnss/tests/`

## Leave This Section When

- leave for [Interfaces](../interfaces/) when the question is what the command
  boundary promises rather than how to change it safely
- leave for [Quality](../quality/) when the operational sequence is clear and
  the next question is proof sufficiency
