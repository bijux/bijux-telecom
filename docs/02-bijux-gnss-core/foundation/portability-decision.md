---
title: Portability Decision
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Portability Decision

The workspace is not pursuing `no_std` as a current repository-wide target.

## Current Decision

- infrastructure and command crates depend on filesystem, process, and OS
  facilities
- the current shared contracts and algorithms prioritize clarity and repository
  ergonomics over embedded portability

## Why This Belongs Here

The question most often starts at the foundational layer: whether core meaning
and record types are intended to anchor a future `no_std` subset. Today the
answer is no, and that decision should be documented with the owner of shared
cross-crate assumptions.

## First Proof Check

- `crates/bijux-gnss-core/Cargo.toml`
- `crates/bijux-gnss-core/docs/BOUNDARY.md`
- `../../03-bijux-gnss-infra/foundation/dependencies-and-adjacencies.md`
