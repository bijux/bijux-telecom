---
title: Release and Versioning
audience: mixed
type: explanation
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
