---
title: Operations
audience: mixed
type: index
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Operations

Open this section when the question is how to change `bijux-gnss-core`
without destabilizing the rest of the repository. Operational guidance here is
about safe maintenance, verification sequence, fixture care, and contract-aware
change practice rather than about runtime deployment.

## Operational Model

```mermaid
flowchart LR
    change["proposed core change"]
    scope["scope and owner check"]
    docs["contract docs update"]
    tests["narrow verification"]
    review["cross-crate review"]
    release["merge-ready change"]

    change --> scope --> docs --> tests --> review --> release
```

The operational burden is heavier here than in most crates because a small core
change can fan out into every higher-level package.

## Read These First

- open [Change Sequence](change-sequence.md) first when you are about to edit a
  public type, invariant, or serialized record
- open [Verification Commands](verification-commands.md) when you need the
  minimum executable proof for a core change
- open [Fixture and Regression Care](fixture-and-regression-care.md) when the
  work touches serialized artifacts or timekeeping regressions

## First Proof Check

- `crates/bijux-gnss-core/README.md`
- `crates/bijux-gnss-core/docs/TESTS.md`
- `crates/bijux-gnss-core/tests/`

## Pages In This Section

- [Common Workflows](common-workflows.md)
- [Local Development](local-development.md)
- [Change Sequence](change-sequence.md)
- [Verification Commands](verification-commands.md)
- [Fixture and Regression Care](fixture-and-regression-care.md)
- [Review Scope](review-scope.md)
- [Release and Versioning](release-and-versioning.md)

## Leave This Section When

- leave for [Interfaces](../interfaces/) when the question is what the crate
  promises rather than how to change it safely
- leave for [Quality](../quality/) when the operational sequence is clear and
  the question becomes proof sufficiency
