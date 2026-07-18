---
title: Dataset Contracts
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-18
---

# Dataset Contracts

Dataset contracts turn repository files into typed capture metadata. They keep
commands, tests, receiver runs, and validation tooling from inventing separate
rules for registry lookup, sidecar loading, coordinates, and recorded capture
provenance.

## Dataset Resolution Flow

```mermaid
flowchart TD
    registry["datasets/registry.toml"]
    entry["DatasetEntry"]
    sidecar["raw-IQ sidecar"]
    metadata["RawIqMetadata"]
    provenance["RecordedCaptureProvenance"]
    caller["command, receiver, test"]

    registry --> entry
    entry --> provenance
    entry --> sidecar --> metadata
    metadata --> caller
    provenance --> caller
```

## Contract Families

| family | owns | first proof |
| --- | --- | --- |
| registry | dataset ids, capture file routes, declared metadata, and dataset entries | `crates/bijux-gnss-infra/src/datasets/registry.rs` |
| sidecar loading | file-backed raw-IQ metadata loading and validation handoff | `crates/bijux-gnss-infra/src/datasets/raw_iq_metadata.rs` |
| metadata resolution | dataset-aware sidecar and explicit metadata resolution | `crates/bijux-gnss-infra/src/datasets/raw_iq_metadata.rs` |
| capture provenance | recorded capture context attached to dataset entries | `crates/bijux-gnss-infra/src/datasets/registry/` |
| coordinate parsing | repository-side ECEF parsing for dataset records | `crates/bijux-gnss-infra/src/parse/coordinates.rs` |

## Boundary Rules

- Infra owns where repository metadata comes from and how callers resolve it.
- Signal owns raw-IQ metadata types, quantization, and sample semantics.
- Receiver owns execution over resolved datasets.
- Command owns operator flags and report rendering, not registry semantics.

## Reader Checks

- Does the dataset id resolve to the same capture meaning from every caller?
- Is sidecar metadata validated once instead of reinterpreted by command code?
- Is coordinate parsing explicit enough to avoid silent frame or unit changes?
- Can the reader see which capture provenance came from the registry and which
  came from runtime execution?

## First Proof Check

Inspect `crates/bijux-gnss-infra/docs/DATASETS.md`,
`crates/bijux-gnss-infra/src/datasets/registry.rs`,
`crates/bijux-gnss-infra/src/datasets/raw_iq_metadata.rs`,
`crates/bijux-gnss-infra/src/parse/coordinates.rs`, and dataset or sidecar
integration tests before changing dataset contract claims.
