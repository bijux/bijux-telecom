---
title: Artifact Contracts
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Artifact Contracts

Artifact contracts are the most obvious persistence-facing surface in
`bijux-gnss-core`.

## What They Own

- `ArtifactHeaderV1`
- `ArtifactV1`
- `ArtifactKind`
- versioned acquisition, observation, tracking, navigation, and support-matrix
  payload families
- payload validation through `ArtifactPayloadValidate` and `ArtifactValidate`
- read-policy and payload-conversion helpers exposed through the curated API

## Why They Matter

These contracts define what downstream crates may persist, inspect, export, and
validate without guessing at internal runtime structures.

## Boundary Rule

The core crate owns envelope meaning and validation rules. It does not own path
names, directory names, or repository storage layout.

## Protecting Proof

- `crates/bijux-gnss-core/src/artifact.rs`
- `crates/bijux-gnss-core/src/artifact/v1.rs`
- `crates/bijux-gnss-core/docs/CONTRACTS.md`
- `crates/bijux-gnss-core/docs/SERIALIZATION.md`
- `crates/bijux-gnss-core/tests/nav_artifact_validation.rs`
- `crates/bijux-gnss-core/tests/tracking_artifact_validation.rs`
