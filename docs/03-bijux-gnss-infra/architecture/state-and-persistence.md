---
title: State and Persistence
audience: mixed
type: explanation
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# State and Persistence

State is the reason this crate exists.

## Main Persisted Families

- dataset registry entries and sidecar-derived metadata
- run identity and directory layout
- manifests, reports, history entries, and artifact headers
- provenance hashes and environment evidence

## Persistence Boundary

Infra owns how repository state is laid out and interpreted. It does not own
the semantic meaning of core artifact payloads or the runtime-side production
of those payloads.

## Durability Rule

Run footprints should remain understandable after the command and process that
created them are gone. That is the architectural standard here, not a nice-to-
have.

## First Proof Check

- `crates/bijux-gnss-infra/docs/RUN_LAYOUT.md`
- `crates/bijux-gnss-infra/src/run_layout/`
- `crates/bijux-gnss-infra/src/hash/`
