---
title: Dependencies And Adjacencies
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# Dependencies And Adjacencies

This page explains which nearby crates `bijux-gnss` depends on directly and
which ones matter as boundary checks.

## Direct Technical Dependencies

- `bijux-gnss-core` for shared contracts and schemas
- `bijux-gnss-signal` for signal-domain surfaces used by commands
- `bijux-gnss-receiver` for runtime execution flows
- `bijux-gnss-infra` for repository-facing workflows and artifact access
- `bijux-gnss-nav` for navigation-backed commands when the feature is enabled

## Adjacencies That Matter More Than The Cargo Graph

- the root README and operator docs depend on this crate being the stable
  public entrypoint
- neighboring handbooks depend on the command crate staying thin enough that
  ownership remains legible

## Practical Reading Rule

If a change needs simultaneous edits in `bijux-gnss` and a lower crate, review
the boundary first. Cross-crate edits often mean the command layer is trying to
own too much or too little.
