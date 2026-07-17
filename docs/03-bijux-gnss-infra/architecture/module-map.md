---
title: Module Map
audience: mixed
type: explanation
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
| `src/datasets/` | registry parsing, raw-IQ metadata resolution, and coordinate parsing |
| `src/run_layout/` | run identity, directories, manifests, reports, history, and persisted records |
| `src/artifact_inspection/` | artifact kind detection, explanation, and persisted validation entrypoints |
| `src/overrides/` | typed receiver-profile override application |
| `src/experiments.rs` and `src/sweep.rs` | experiment specs and sweep expansion |
| `src/hash/` | provenance-oriented hashing and environment capture |
| `src/parse/` | small infrastructure parsing helpers |
| `src/validate_reference.rs` | repository-facing validation-reference adapters |
| `src/commands.rs` | helper assembly used by higher-level infrastructure callers |

## Why This Map Exists

Without a clear map, the crate starts to look like "whatever touches the
repository." The point is the opposite: each area owns one durable repository
contract family.

## First Proof Check

- `crates/bijux-gnss-infra/src/`
- `crates/bijux-gnss-infra/docs/ARCHITECTURE.md`
