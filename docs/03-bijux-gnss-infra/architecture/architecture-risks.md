---
title: Architecture Risks
audience: mixed
type: explanation
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Architecture Risks

The biggest structural risk in `bijux-gnss-infra` is honest overreach. The
crate sits at a legitimate aggregation boundary, so almost every convenience
argument sounds plausible.

## Main Risks

- glue-bucket growth:
  unrelated helpers pile in because the crate already touches many surfaces
- boundary confusion:
  payload meaning from core or runtime behavior from receiver gets silently
  redefined here
- hidden command policy:
  command-specific workflow choices harden into repository contracts
- unstable persisted footprint:
  manifests, reports, or histories drift without being treated as durable
  contracts

## Risk Response

- keep repository ownership explicit in docs and module layout
- treat run footprints and dataset interpretation as contracts, not casual
  implementation detail
- default ambiguous behavior toward the stronger product owner crate

## First Proof Check

- `crates/bijux-gnss-infra/docs/ARCHITECTURE.md`
- `crates/bijux-gnss-infra/tests/integration_guardrails.rs`
