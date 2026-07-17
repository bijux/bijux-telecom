---
title: Change Principles
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# Change Principles

Changes to `bijux-gnss-nav` should preserve scientific legibility, not only
compile or benchmark behavior.

## Principles

- prefer exposing scientific families through durable public surfaces rather
  than making downstream crates reach into internal files
- keep parser, correction, and estimator law near the families that own it
  instead of centralizing everything in one convenience module
- add runtime or repository integration only through explicit seams, never by
  letting those concerns backfill into solver code
- treat refusal paths and integrity evidence as first-class outcomes, not as
  second-order error handling
- widen public API only when multiple downstream owners genuinely need a stable
  scientific contract

## Warning Signs

- a new helper is easier to describe by its caller than by its scientific role
- a solver change adds file-path or command-default knowledge
- product parsing begins to depend on repository layout assumptions
