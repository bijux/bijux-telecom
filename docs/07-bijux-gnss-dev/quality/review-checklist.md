---
title: Review Checklist
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Review Checklist

Use this checklist when reviewing a `bijux-gnss-dev` change.

## Checklist

- is the owning maintainer workflow named clearly
- does the change preserve the boundary against product behavior
- if a new file is read or written, is that governed location documented
- if a command meaning changed, do the handbook pages still match it
- if benchmark behavior changed, is the evidence path still explicit
- if a test changed, does it still defend a maintainer contract rather than
  incidental implementation detail

## First Proof Check

Inspect `crates/bijux-gnss-dev/docs/WORKFLOWS.md`,
`crates/bijux-gnss-dev/docs/GOVERNANCE_FILES.md`,
`crates/bijux-gnss-dev/docs/OUTPUTS.md`, and
`crates/bijux-gnss-dev/docs/TESTS.md` first. Then inspect the changed command
or test path so the checklist is answered with repository-backed evidence.
