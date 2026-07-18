---
title: Measurement And Engine Contracts
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-18
---

# Measurement And Engine Contracts

`bijux-gnss-core` owns the record meanings that must survive handoff between
signal, receiver, navigation, infrastructure, and command crates. These records
are not stage implementations; they are the shared language used by stage
implementations.

## Shared Measurement Flow

```mermaid
flowchart LR
    samples["SamplesFrame"]
    acquisition["AcqRequest and AcqResult"]
    tracking["tracking epochs and transitions"]
    observations["ObsEpoch and metadata"]
    navigation["NavSolutionEpoch and residuals"]
    artifacts["ArtifactV1 envelopes"]

    samples --> acquisition --> tracking --> observations --> navigation --> artifacts
    observations --> artifacts
```

## Contract Families

| family | owns | first proof |
| --- | --- | --- |
| sample bridge | sample-frame records that cross source, signal, and receiver boundaries | `crates/bijux-gnss-core/src/observation/`, `crates/bijux-gnss-core/docs/CONTRACTS.md` |
| acquisition records | request/result records and explainability payload meaning | `crates/bijux-gnss-core/src/observation/` |
| tracking records | tracking epochs, transitions, uncertainty, and channel-state language | `crates/bijux-gnss-core/src/observation/` |
| observation records | `ObsEpoch`, metadata, receiver roles, timing, rejection, and uncertainty classes | `crates/bijux-gnss-core/src/observation/` |
| navigation-solution records | solution epochs, residuals, validity, lifecycle, and refusal classes | `crates/bijux-gnss-core/src/nav_solution.rs` |
| artifact envelopes | versioned cross-crate artifact headers, payload kinds, and validation traits | `crates/bijux-gnss-core/src/artifact.rs`, `crates/bijux-gnss-core/src/artifact/` |

## Boundary Rules

- Core owns what exchanged records mean.
- Signal owns reusable signal computation before receiver stage execution.
- Receiver owns stage ordering and runtime artifacts that use these records.
- Navigation owns solver behavior that consumes and emits these records.
- Infra owns repository persistence around these records.
- Command owns operator routes and reports over these records.

## Reader Checks

- Is the record shared by more than one crate?
- Would changing it alter persisted artifact interpretation or downstream
  solver assumptions?
- Can higher crates use the record without importing runtime, filesystem, or
  command policy?
- Does the proof live in core tests or in the crate that implements behavior
  over the record?

## First Proof Check

Inspect `crates/bijux-gnss-core/docs/CONTRACTS.md`,
`crates/bijux-gnss-core/docs/CONTRACT_MAP.md`,
`crates/bijux-gnss-core/docs/SERIALIZATION.md`,
`crates/bijux-gnss-core/src/artifact.rs`,
`crates/bijux-gnss-core/src/artifact/`,
`crates/bijux-gnss-core/src/observation/`, and
`crates/bijux-gnss-core/src/nav_solution.rs`.
