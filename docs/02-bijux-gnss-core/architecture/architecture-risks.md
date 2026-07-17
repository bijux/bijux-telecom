---
title: Architecture Risks
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Architecture Risks

The biggest structural risks in `bijux-gnss-core` come from success. Because
many crates depend on it, every convenience argument sounds plausible.

## Main Risks

- contract sprawl:
  too many marginally shared types weaken the meaning layer
- public-surface creep:
  helpers leak into `api.rs` because they are useful, not because they are
  durable contracts
- boundary confusion:
  artifact-envelope concerns get mixed with filesystem or runtime concerns
- dependency inversion:
  core starts importing higher-level logic under the label of "shared helper"

## Risk Response

- keep `api.rs` curated and guardrailed
- keep contract-family docs and invariants updated in the same change sets
- default ambiguous additions toward a stronger downstream owner

## First Proof Check

- `crates/bijux-gnss-core/tests/public_api_guardrail.rs`
- `crates/bijux-gnss-core/tests/integration_guardrails.rs`
- `crates/bijux-gnss-core/docs/ARCHITECTURE.md`
