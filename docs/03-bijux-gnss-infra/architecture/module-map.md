---
title: Module Map
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Module Map

The module map matters because `bijux-gnss-infra` groups several repository
subsystems that would become a catch-all quickly if their ownership were vague.

## Main Ownership Map

| code area | primary ownership |
| --- | --- |
| `src/api.rs` | curated infrastructure-facing surface |
| `src/datasets/registry.rs` plus `src/datasets/registry/` | dataset registry loading, entry parsing, and path resolution |
| `src/datasets/raw_iq_metadata.rs` plus `src/datasets/raw_iq_metadata/` | sidecar loading, metadata resolution, and capture validation |
| `src/run_layout.rs` plus `src/run_layout/` | run-layout surface assembly, identity, directories, paths, persistence, provenance, and persisted records |
| `src/artifact_inspection/` plus `src/artifact_inspection/validation/` | artifact kind detection, explanation, summary reporting, and persisted validation entrypoints |
| `src/overrides/receiver_profile.rs` and `src/overrides/sweep_parameters.rs` | typed receiver-profile and sweep-parameter override application |
| `src/experiments.rs` and `src/sweep.rs` | experiment specs and sweep expansion |
| `src/hash/mod.rs` and `src/hash/provenance.rs` | provenance-oriented hashing and environment capture |
| `src/parse/` and `src/parse/coordinates.rs` | small infrastructure parsing helpers |
| `src/validate_reference.rs` | repository-facing validation-reference adapters |
| `src/commands.rs` | helper assembly used by higher-level infrastructure callers |

## Why This Map Exists

Without a clear map, the crate starts to look like "whatever touches the
repository." The point is the opposite: each area owns one durable repository
contract family.

## First Proof Check

- `crates/bijux-gnss-infra/src/api.rs`
- `crates/bijux-gnss-infra/src/datasets/`
- `crates/bijux-gnss-infra/src/run_layout.rs`
- `crates/bijux-gnss-infra/src/run_layout/`
- `crates/bijux-gnss-infra/src/artifact_inspection/summary.rs`
- `crates/bijux-gnss-infra/docs/ARCHITECTURE.md`
