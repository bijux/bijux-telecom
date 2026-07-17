---
title: Review Scope
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# Review Scope

Reviewers should scope a `bijux-gnss` change by command family first, then by
file count.

## Narrow Review

- one command family
- one reporting path
- one command/runtime support helper
- one facade maintenance change with clear lower-owner provenance

## Broad Review Triggers

- changes to command names, flags, or output shape
- changes that span several command families
- changes to `src/lib.rs`
- changes that require synchronized edits in `receiver`, `infra`, `nav`,
  `signal`, or `core`

## Why File Count Is Misleading

A one-line change to a stable command flag or validation publication path can
be riskier than a larger refactor isolated inside one support module.
