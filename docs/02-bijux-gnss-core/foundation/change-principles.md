---
title: Change Principles
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Change Principles

Changes in `bijux-gnss-core` should be rarer and more deliberate than changes
in higher-level crates, because contract churn multiplies downstream review
cost.

## Principles

1. Prefer extending an existing contract family over inventing a new public
   namespace.
2. Keep implementation helpers private unless two or more downstream crates
   truly need the shared meaning.
3. When serialized meaning changes, update contract prose, invariants, and
   validation evidence in the same change set.
4. Add behavior elsewhere before moving it into core; make the shared need
   prove itself.
5. If a type is only convenient for one owner today, keep it with that owner
   until the cross-crate contract need is clear.

## What A Good Core Change Looks Like

- a new observation field with obvious cross-crate meaning
- a new artifact payload version with explicit validation rules
- a clarified unit or time conversion invariant backed by tests

## What A Bad Core Change Looks Like

- a receiver helper turned public because one CLI flow also wants it
- a persistence-specific record added because it was easier than defining it in
  infrastructure
- a navigation-solver record generalized before a second consumer exists

## First Proof Check

- `crates/bijux-gnss-core/docs/CHANGE_RULES.md`
- `crates/bijux-gnss-core/tests/public_api_guardrail.rs`
