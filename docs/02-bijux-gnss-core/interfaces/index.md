---
title: Interfaces
audience: mixed
type: index
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Interfaces

Open this section when the question is contractual: which public imports,
artifact envelopes, records, and validation-facing shapes are safe for another
crate or tool to rely on.

This section is where the curated `bijux_gnss_core::api` surface, artifact
payload families, observation records, navigation-solution records, and config
validation contracts become explicit.

## First Proof Check

- `crates/bijux-gnss-core/src/api.rs`
- `crates/bijux-gnss-core/docs/PUBLIC_API.md`
- `crates/bijux-gnss-core/docs/CONTRACTS.md`
- `crates/bijux-gnss-core/docs/SERIALIZATION.md`

## Leave This Section When

- leave for [Foundation](../foundation/) when the contract dispute is really a
  package-boundary dispute
- leave for [Architecture](../architecture/) when the interface issue reveals
  structural drift underneath it
- leave for [Operations](../operations/) or [Quality](../quality/) when the
  contract is clear and the question becomes safe change or proof
