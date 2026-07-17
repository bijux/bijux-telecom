---
title: Invariants
audience: mixed
type: explanation
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

## Boundary Invariants

- infra should remain an infrastructure owner rather than drifting into
  receiver, nav, signal, or command policy

## Protecting Proof

- `crates/bijux-gnss-infra/docs/RUN_LAYOUT.md`
- `crates/bijux-gnss-infra/docs/DATASETS.md`
- `crates/bijux-gnss-infra/tests/`
