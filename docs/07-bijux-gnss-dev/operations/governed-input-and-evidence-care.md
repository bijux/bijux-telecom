---
title: Governed Input And Evidence Care
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Governed Input And Evidence Care

The reviewed files and evidence paths this crate touches are part of the
maintainer contract, not incidental implementation detail.

## Care Rules

- change governed-input expectations only when the repository truly changed its
  reviewed policy
- keep evidence outputs in the documented governed locations
- explain why a policy or benchmark-evidence change is credible, not just why
  it makes a command pass
- update the handbook when a workflow gains a new reviewed file or evidence
  location

## Boundary Rule

This crate may validate reviewed files and write maintenance evidence, but it
should not quietly invent new repository control files or unmanaged output
paths.

## First Proof Check

- `crates/bijux-gnss-dev/docs/GOVERNANCE_FILES.md`
- `crates/bijux-gnss-dev/docs/OUTPUTS.md`
- `crates/bijux-gnss-dev/src/main.rs`
