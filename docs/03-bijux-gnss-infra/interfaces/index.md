---
title: Interfaces
audience: mixed
type: index
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Interfaces

Open this section when the question is contractual: which repository-facing
records, manifests, overrides, validation adapters, and public imports are safe
for another crate or tool to rely on.

## First Proof Check

- `crates/bijux-gnss-infra/src/api.rs`
- `crates/bijux-gnss-infra/docs/CONTRACTS.md`
- `crates/bijux-gnss-infra/docs/DATASETS.md`
- `crates/bijux-gnss-infra/docs/RUN_LAYOUT.md`

## Leave This Section When

- leave for [Foundation](../foundation/) when the contract dispute is really a
  package-boundary dispute
- leave for [Architecture](../architecture/) when the interface issue reveals
  structural drift underneath it
- leave for [Operations](../operations/) or [Quality](../quality/) when the
  contract is clear and the question becomes safe change or proof
