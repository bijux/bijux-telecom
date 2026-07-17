---
title: Common Workflows
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Common Workflows

## Typical Change Families

- update audit-exception validation rules
- update deviation-governance validation rules
- change derived audit-ignore argument behavior
- extend or adjust benchmark comparison behavior
- strengthen repository-structure guardrail tests
- update slow-test roster policy through guarded tests rather than through a
  new command flag

## Workflow Rule

Choose the workflow based on the owned maintainer surface. A benchmark-evidence
change is reviewed differently from a governed-TOML validation change even if
both happen in the same binary. The audit and deviation workflows are read-only
contract checks; `bench-compare` is the write-producing evidence workflow.
