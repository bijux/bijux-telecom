---
title: Dependencies and Adjacencies
audience: mixed
type: explanation
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Dependencies and Adjacencies

`bijux-gnss-infra` is allowed to aggregate lower-level product APIs, but only
to define repository-facing contracts over them.

## Dependency Direction

- infra may depend on `core`, `signal`, `nav`, and `receiver` when repository
  state needs a typed bridge to those contracts
- higher-level crates such as `bijux-gnss` and tooling consume infra’s
  repository-facing surface

## Adjacency Pressure

- `receiver` pulls on infra through artifacts, profiles, and validation seams
- `signal` pulls on infra through raw-IQ metadata and sidecar interpretation
- `nav` pulls on infra through persisted-reference validation surfaces
- `gnss` pulls on infra through workflow preparation and run persistence

## Dependency Smells

- importing a product crate only to hide one command convenience here
- exposing a lower-level API unchanged without adding repository ownership
- adding helpers that neither own repository state nor strengthen a repository
  contract

## First Proof Check

- `crates/bijux-gnss-infra/docs/ARCHITECTURE.md`
- `crates/bijux-gnss-infra/tests/integration_guardrails.rs`
