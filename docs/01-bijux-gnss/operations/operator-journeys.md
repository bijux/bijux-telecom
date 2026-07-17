---
title: Operator Journeys
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# Operator Journeys

This page absorbs the older root-level "getting started" and "end-to-end"
guides into the command owner that actually presents those workflows.

## Ingest-First Journey

The canonical first journey is still:

1. validate the raw-IQ metadata or sidecar
2. inspect a registered dataset or fixture
3. run the relevant ingest or pipeline workflow
4. inspect emitted evidence or diagnostics

The likely owning command surfaces behind that route are:

1. `validate` for sidecar or metadata checks
2. `artifact` or dataset-facing inspection helpers for repository inputs
3. `ingest`, `synthetic`, or another pipeline-owning command family for the
   actual run
4. `diagnostics` or report-facing output helpers for operator interpretation

That sequence belongs here because it is an operator-facing route across lower
owners, not a lower-owner contract by itself.

## End-To-End Rule

If a journey spans ingest, run, inspect, validate, diagnose, or replay from the
top-level command tree, `bijux-gnss` owns the route description. The moment the
reader needs the internal semantics of one stage, hand off to the owning crate
handbook.

## Reader Boundary

This page should tell an operator which command route to open next and which
crate handbook owns the next proof. It should not retell receiver-stage math,
dataset persistence internals, or navigation-science semantics.
