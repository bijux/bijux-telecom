---
title: Release and Versioning
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Release and Versioning

Versioning pressure in infra comes from persisted repository behavior more than
from algorithmic novelty.

## Versioning Rules

- prefer additive evolution when possible
- treat manifest, report, and history semantics as release-significant
- document any repository-footprint shift explicitly
- avoid calling a footprint or dataset-interpretation change "internal cleanup"

## Release Smell

If a release note would hide a changed run footprint behind generic wording,
the repository contract is being described dishonestly.

## First Proof Check

Inspect `crates/bijux-gnss-infra/docs/RUN_LAYOUT.md`,
`crates/bijux-gnss-infra/docs/DATASETS.md`,
`crates/bijux-gnss-infra/docs/VALIDATION.md`, and
`crates/bijux-gnss-infra/src/api.rs`. Then inspect the matching tests or source
families to confirm any claimed release safety still matches persisted
repository behavior.
