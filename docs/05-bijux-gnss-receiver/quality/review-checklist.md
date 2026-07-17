---
title: Review Checklist
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Review Checklist

Use this checklist when reviewing a receiver change.

## Checklist

- is the owning runtime family named clearly
- does the change preserve the crate boundary against command, repository, or
  science creep
- if `api.rs` changed, is the public promise durable
- do artifacts, reports, or validation outputs still mean the same thing
- did the author run the narrowest honest validation commands
- if a fixture or synthetic expectation changed, is the reason explicit and
  technically credible
- do the handbook pages still match the changed behavior
