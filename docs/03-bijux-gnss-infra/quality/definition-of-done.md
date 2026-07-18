---
title: Definition of Done
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Definition of Done

An infra change is done when the repository evidence contract remains legible,
deterministic, and owned by infra for the right reason. The change must explain
what repository state means after execution, not merely where a helper lives.

## Completion Gate

| changed surface | done means | proof to start from |
| --- | --- | --- |
| dataset registry or capture metadata | dataset identity and recorded provenance are deterministic and reader-visible | `crates/bijux-gnss-infra/docs/DATASETS.md` plus changed source proof |
| run layout, manifest, report, or history | persisted execution footprint stays understandable after the producing command is gone | `crates/bijux-gnss-infra/docs/RUN_LAYOUT.md` plus changed source proof |
| artifact inspection | persisted artifacts are identified and explained without taking over producer meaning | `crates/bijux-gnss-infra/docs/CONTRACTS.md` and artifact-inspection source proof |
| overrides or sweeps | configuration mutation remains typed, reviewable, and reproducible | `cargo test -p bijux-gnss-infra --test integration_overrides` |
| validation adapter or hashing | repository-side validation and provenance stay explicit and reproducible | `crates/bijux-gnss-infra/docs/VALIDATION.md`, `HASHING.md`, and focused source proof |

## Not Done Yet Means

- a new helper exists only because another owner did not want to hold it
- run-footprint meaning changed but the docs still describe the old behavior
- the repository trust story depends on assumptions that are no longer written
  down
- a product claim moved into infra without a lower-owner test
- a persisted shape changed without naming how old readers should interpret it

## Reader Questions Before Commit

- What repository state is now created, read, validated, or explained?
- Which command or runtime surface produces the state, and which surface later
  consumes it?
- Is infra defining persistence and inspection, or is it accidentally defining
  product behavior?
- What exact test, fixture, or source inspection would catch a contract drift?

An infra change is not complete until contract prose, source behavior, and
available proof describe the same repository state. If dedicated proof is
absent, say that directly in the change and keep the claim narrow.
