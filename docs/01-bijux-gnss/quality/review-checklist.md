---
title: Review Checklist
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# Review Checklist

Use this checklist when reviewing a `bijux-gnss` change.

## Checklist

- is the owning command family named clearly
- does the change preserve the crate boundary against runtime, repository, or
  science creep
- if the binary surface changed, is the public promise durable
- if `src/lib.rs` changed, does the facade still stay thin
- did the author run the narrowest honest validation commands
- if a workflow fixture changed, is the reason explicit and technically
  credible
- do the handbook pages still match the changed behavior

## Protecting Proof

- `crates/bijux-gnss/docs/TESTS.md`
- `crates/bijux-gnss/docs/FACADE.md`
- `docs/01-bijux-gnss/this-package-does-not-own.md`
