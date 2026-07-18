---
title: Ownership Boundary
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Ownership Boundary

`bijux-gnss-receiver` sits above `bijux-gnss-core`, `bijux-gnss-signal`, and
optionally `bijux-gnss-nav`, while serving `bijux-gnss` and
`bijux-gnss-infra`.

## Upstream Dependencies It May Assume

- `bijux-gnss-core` provides shared contracts, units, artifacts, and time or
  observation meaning that the receiver consumes
- `bijux-gnss-signal` provides signal primitives and reusable DSP-domain logic
  the receiver stages call into
- `bijux-gnss-nav` provides navigation science, estimators, and correction law
  when the `nav` feature is enabled

## Downstream Packages It Serves

- `bijux-gnss` consumes the receiver surface to expose commands and workflows
- `bijux-gnss-infra` consumes receiver artifacts and runtime-side validation
  outputs before persisting and indexing them

## Boundary Tests

- if a concern starts from stage ordering, runtime state, or side-effectful
  source and sink handling, it can belong here
- if a concern starts from signal math that should be reusable outside the
  runtime, it does not belong here
- if a concern starts from repository layout or persisted report naming, it
  does not belong here
- if a concern changes how acquisition, tracking, or observation stages are
  composed into a run, it does belong here

## Where Boundary Drift Usually Starts

- stage wrappers begin re-implementing signal or navigation science locally
- runtime types begin carrying repository file layout assumptions
- command-facing defaults or formatting rules begin leaking into receiver
  configuration surfaces
