---
title: Review Scope
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Review Scope

Reviewers should scope a receiver change by runtime family first, then by file
count.

## Narrow Review

- one runtime configuration or diagnostics surface
- one stage-local behavior inside acquisition, tracking, or observations
- one port seam or adapter
- one receiver-owned validation or synthetic proof family

## Broad Review Triggers

- changes to `src/api.rs`
- changes that span more than one stage family
- changes to artifact shape, support matrices, or validation-report meaning
- changes that require synchronized edits in `gnss`, `infra`, `signal`, or
  `nav`

## Why File Count Is Misleading

A one-line threshold change in acquisition acceptance or validation policy can
be riskier than a larger refactor isolated inside one stage subsystem.
