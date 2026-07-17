---
title: Invariants
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Invariants

The most important infra invariants are about shared interpretation of
repository state.

## Dataset Invariants

- registry and sidecar interpretation should not vary silently by caller
- coordinate parsing should stay one shared infrastructure rule, not a per-tool
  improvisation

## Run-Footprint Invariants

- the same run context should resolve to the same directory footprint
- manifests, reports, and history entries should remain understandable after
  the producing command is gone

## Variation Invariants

- typed overrides and sweeps should remain more explicit than raw string
  mutation

## Provenance Invariants

- repository-facing hashes and front-end provenance should keep describing run
  preparation and environment evidence rather than drifting into product
  identity

## Boundary Invariants

- infra should remain an infrastructure owner rather than drifting into
  receiver, nav, signal, or command policy

## Protecting Proof

- `crates/bijux-gnss-infra/docs/RUN_LAYOUT.md`
- `crates/bijux-gnss-infra/docs/DATASETS.md`
- `crates/bijux-gnss-infra/docs/HASHING.md`
- `crates/bijux-gnss-infra/src/datasets/registry.rs`
- `crates/bijux-gnss-infra/src/datasets/raw_iq_metadata.rs`
- `crates/bijux-gnss-infra/src/parse/coordinates.rs`
- `crates/bijux-gnss-infra/src/hash/provenance.rs`
- `crates/bijux-gnss-infra/src/run_layout/provenance/front_end.rs`
- `crates/bijux-gnss-infra/tests/integration_overrides.rs`
- `crates/bijux-gnss-infra/tests/integration_guardrails.rs`
