---
title: Change Validation
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Change Validation

Choose the smallest honest proof set that exercises the changed owner.

## Minimum Validation By Change Type

- catalog change:
  run registry and wavelength-oriented proof
- code-family change:
  run the matching reference and continuity tests for that family
- DSP change:
  run the matching long-duration, spectrum, or tracking proof
- raw-IQ or sample change:
  run metadata and conversion proof
- validation change:
  run observation-validation property and integration proof

## Bad Validation Patterns

- running only one unrelated integration test because it is convenient
- relying on a receiver-level test to prove signal truth
- updating reference data without proving why the canonical behavior changed

## First Proof Check

Use `crates/bijux-gnss-signal/docs/TESTS.md` as the validation map. Then
inspect the matching reference, continuity, spectrum, metadata, or property
tests under `crates/bijux-gnss-signal/tests/` before declaring the proof set
complete.
