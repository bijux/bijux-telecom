---
title: Risk Register
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Risk Register

## Glue-Bucket Growth

Risk:
infra keeps accumulating helpers because it already touches many owners.

Response:
keep the ownership tests from the foundation section active in review.

## Silent Run-Footprint Drift

Risk:
manifests, reports, or history semantics change without being treated as a
durable contract shift.

Response:
tie footprint changes to explicit documentation updates in the same change set.

## Dataset Interpretation Drift

Risk:
different callers begin to treat the same registry or sidecar state
differently.

Response:
keep one typed dataset contract and review changes as shared repository
behavior, not local fixes.

## Validation Boundary Confusion

Risk:
infra starts to redefine runtime validation instead of owning only the
persisted-evidence side.

Response:
keep runtime behavior with receiver and repository interpretation with infra.
