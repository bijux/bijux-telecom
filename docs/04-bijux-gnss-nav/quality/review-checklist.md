---
title: Review Checklist
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# Review Checklist

Use this checklist when reviewing a nav change.

## Checklist

- is the owning scientific family named clearly
- does the change preserve the crate boundary against runtime or repository
  creep
- if `api.rs` changed, is the public promise durable
- do refusal, rejection, downgrade, or integrity outcomes still make sense
- did the author run the narrowest honest validation commands
- if a reference or fixture changed, is the reason explicit and scientifically
  credible
- do the handbook pages still match the changed behavior

## Protecting Proof

- `crates/bijux-gnss-nav/docs/TESTS.md`
- `crates/bijux-gnss-nav/docs/PUBLIC_API.md`
- `docs/04-bijux-gnss-nav/this-package-does-not-own.md`
