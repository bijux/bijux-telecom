---
title: Risk Register
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Risk Register

## Contract Surface Creep

Risk:
more helpers become public because downstream code finds them convenient.

Response:
keep `api.rs` curated and guardrailed.

## False Shared Ownership

Risk:
runtime or repository behavior gets generalized into core too early.

Response:
apply the ownership questions from the foundation section before adding new
families.

## Silent Serialization Drift

Risk:
fixtures keep passing while serialized meaning quietly changes.

Response:
tie fixture updates to documentation and validation updates in the same change
set.

## Weak Downstream Assumption Tracking

Risk:
higher-level crates start depending on behavior that no invariant page names.

Response:
write the invariant or narrow the public promise.
