---
title: Execution Model
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# Execution Model

`bijux-gnss-nav` is computational, but it is not one opaque compute kernel. It
executes as a sequence of scientific interpretations.

## Normal Flow

1. external product bytes or text enter through a format family
2. typed navigation state is produced for a constellation or reference product
3. correction law interprets observations and product state together
4. one estimator family consumes those inputs and emits a solution, refusal, or
   evidence record
5. downstream packages decide when and where that scientific result is used

## Important Architectural Distinction

The crate may own runtime-neutral helpers such as `NavigationEngine` and
`PositionRuntime`, but that does not make it the owner of live receiver
orchestration. Those types represent navigation behavior, not stage scheduling.

## Families With Separate Execution Logic

- product parsing families execute per format family and constellation
- position solvers execute around observation sets and correction chains
- PPP executes as a stateful filter with precise-product policy
- RTK executes through alignment, differencing, ambiguity handling, and
  baseline-quality evaluation

## Why This Matters

Treating all of this as "just nav execution" hides where correctness actually
comes from and makes boundary reviews weaker.
