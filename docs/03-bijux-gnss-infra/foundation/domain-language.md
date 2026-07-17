---
title: Domain Language
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Domain Language

The infrastructure crate has its own vocabulary, and it is different from the
signal, navigation, and receiver vocabularies nearby.

## Vocabulary Families

- dataset language:
  registry entries, capture metadata, sidecars, coordinates, provenance
- run-footprint language:
  run identity, manifests, reports, directories, history entries
- variation language:
  common overrides, sweep parameters, experiment specs, expanded run variants
- validation language:
  artifact inspection, validation summaries, reference adapters
- provenance language:
  hashes, git state, CPU features, reproducibility evidence

## Why This Page Exists

Infra looks like glue when the vocabulary is not named. Once the language is
explicit, the crate becomes easier to keep honest and harder to bloat with
unowned convenience helpers.

## First Proof Check

- `crates/bijux-gnss-infra/src/datasets/`
- `crates/bijux-gnss-infra/src/run_layout/`
- `crates/bijux-gnss-infra/src/overrides/`
- `crates/bijux-gnss-infra/src/hash/`
