---
title: Definition Of Done
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Definition Of Done

A receiver change is done when a reviewer can name the runtime contract that
moved, inspect the evidence that protects it, and see that neighboring crate
ownership was not blurred to make the change easier.

## Completion Gate

| changed area | done means | minimum reader evidence |
| --- | --- | --- |
| acquisition | candidate selection, uncertainty, assistance, and explainability remain coherent before tracking consumes them | targeted acquisition test plus any affected integration scenario |
| tracking | lock state, carrier/code continuity, uncertainty, and handoff remain explainable per epoch | tracking integration proof and channel-state report proof when state meaning moves |
| observations | pseudorange, carrier phase, residual, smoothing, and rejection metadata remain traceable to tracking input | observation unit or integration proof with metadata assertions |
| navigation handoff | receiver-side handoff does not redefine nav science or hide solver assumptions | receiver navigation test plus nav-owner proof when scientific model meaning changes |
| artifacts and diagnostics | emitted run products explain runtime behavior without taking over infra persistence | artifact or diagnostic test plus handbook update when public meaning changes |

## Required Review Questions

- Which receiver family owns the change: `engine`, `pipeline`, `ports`,
  `artifacts`, `reference_validation`, or `sim`?
- Which lower owner supplies meaning: `bijux-gnss-core`,
  `bijux-gnss-signal`, or `bijux-gnss-nav`?
- Which higher owner consumes the result: `bijux-gnss` command UX or
  `bijux-gnss-infra` persistence?
- What exact test or artifact would fail if this contract regressed?

## Proof Anchors

- `crates/bijux-gnss-receiver/docs/TESTS.md` for the receiver test family map.
- `crates/bijux-gnss-receiver/docs/RUNTIME.md` for engine and runtime
  composition.
- `crates/bijux-gnss-receiver/docs/PIPELINE.md` for stage ordering and
  handoff.
- `crates/bijux-gnss-receiver/docs/ARTIFACTS.md` for emitted runtime output.
- `crates/bijux-gnss-receiver/docs/REFERENCE_VALIDATION.md` for bounded truth
  comparison.

Do not call a receiver change done because a broad test happened to pass. Call
it done when the narrowest meaningful proof and the handbook agree on the same
runtime claim.
