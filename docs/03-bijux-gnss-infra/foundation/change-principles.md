---
title: Change Principles
audience: mixed
type: explanation
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Change Principles

Infrastructure changes should make repository behavior more explicit, not more
magical.

## Principles

1. Prefer typed repository contracts over shell-assembled conventions.
2. Keep persisted run footprints understandable after the producing command is
   gone.
3. When a manifest, report, or history meaning changes, update the relevant
   contract docs in the same change set.
4. Let product crates own product behavior even when infra needs to wrap or
   validate it.
5. If a helper adds no repository-facing ownership, it probably belongs
   elsewhere.

## What A Good Infra Change Looks Like

- a clearer run-layout record with aligned docs and tests
- a typed dataset metadata rule that multiple callers can now share
- a reproducibility hash or validation adapter that makes repository evidence
  stronger without changing product semantics

## What A Bad Infra Change Looks Like

- a receiver default moved here because experiments touched it
- a command-specific workaround stored as a generic repository contract
- an artifact interpretation path that quietly redefines payload meaning owned
  by core

## First Proof Check

- `crates/bijux-gnss-infra/docs/RUN_LAYOUT.md`
- `crates/bijux-gnss-infra/docs/CONTRACTS.md`
