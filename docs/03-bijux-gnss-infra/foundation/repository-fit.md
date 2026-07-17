---
title: Repository Fit
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Repository Fit

`bijux-gnss-infra` lives at the seam between product crates and repository
state.

## Fit In The Repository

- `bijux-gnss` calls into infra when command workflows need datasets, run
  layouts, manifests, or infrastructure validation
- `bijux-gnss-receiver` provides runtime artifacts and configurations that infra
  persists, explains, or varies
- `bijux-gnss-nav` participates indirectly when persisted references or
  validation flows need navigation-side knowledge
- `bijux-gnss-signal` defines sample and metadata types that infra resolves from
  repository sidecars
- `bijux-gnss-core` provides the shared records and validation-report language
  that infra stores and interrogates

## Why This Fit Matters

Infra should feel close to many owners without becoming a second home for their
behavior. Its value is shared repository interpretation, not product authority.

## First Proof Check

- repository `Cargo.toml`
- `crates/bijux-gnss-infra/src/api.rs`
- `crates/bijux-gnss-infra/tests/integration_guardrails.rs`
