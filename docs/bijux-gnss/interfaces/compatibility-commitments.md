---
title: Compatibility Commitments
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# Compatibility Commitments

This page records what the command handbook treats as a durable public promise.

## Durable Commitments

- the `bijux` binary remains the primary public surface
- stable command and report families remain organized by operator role rather
  than transient file layout
- the facade remains visibly thin and tied to package convenience
- validation and workflow surfaces stay identifiable as command-boundary
  orchestration rather than ownership transfer

## Allowed Internal Freedom

- internal file reshaping behind stable command behavior
- deeper decomposition of command handlers, support helpers, or reporting logic
  when the public boundary stays clearer
- narrow facade maintenance when lower-owner re-exports remain the same

## Compatibility Review Trigger

If a change alters what an operator or Rust consumer can truthfully infer from
the public command or facade surface, not merely how it is implemented,
compatibility review is required.
