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

## Decision Flow

```mermaid
flowchart LR
    contract["shared contract"]
    std["std-backed workspace"]
    infra["filesystem and process users"]
    future["future portability review"]

    contract --> std
    std --> infra
    contract --> future
```

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

## What This Decision Allows

- Core contracts may use the standard library when that keeps meaning explicit.
- Serialization and validation code may optimize for repository evidence first.
- Downstream crates do not need compatibility shims for embedded builds.

## What Would Reopen The Decision

| trigger | required review |
| --- | --- |
| a supported embedded target appears | identify which core contracts must be portable |
| a crate requests `alloc`-only contracts | separate data shape from repository behavior |
| signal or nav algorithms need standalone reuse | prove the algorithms can remain independent from infra |

## First Proof Check

- `crates/bijux-gnss-core/Cargo.toml`
- `crates/bijux-gnss-core/docs/BOUNDARY.md`
- `../../03-bijux-gnss-infra/foundation/dependencies-and-adjacencies.md`

## Review Checks

- Does the change assume portability that the repository has not accepted?
- Is standard-library use attached to explicit repository or contract value?
- Would a future portability subset know which owner must change first?
