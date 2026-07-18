---
title: Scope And Non-Goals
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Scope And Non-Goals

This page states what `bijux-gnss-receiver` is allowed to own and what it
should refuse even when the runtime depends heavily on those neighboring
surfaces.

## In Scope

- runtime configuration and execution state
- stage ordering and handoff across acquisition, tracking, observations, and
  optional navigation
- runtime source, sink, clock, trace, and metrics seams
- in-memory run artifacts and receiver-boundary validation helpers
- synthetic runtime helpers that prove receiver behavior

## Explicit Non-Goals

- repository file discovery, run directories, and persisted manifests
- command parsing, report phrasing, and operator workflow policy
- reusable signal primitives and low-level DSP ownership
- standalone navigation algorithms, format parsing, and precise-product
  interpretation
- generic shared contracts without receiver-runtime meaning

## Why These Refusals Matter

If the receiver crate absorbs repository or command policy, runtime behavior
becomes harder to inspect. If it absorbs signal or navigation science,
downstream review loses the ability to challenge claims at the right boundary.

## Strong Negative Test

If a change cannot be explained in terms of runtime composition, stage
orchestration, runtime seams, or receiver-boundary artifacts, it probably does
not belong here.
