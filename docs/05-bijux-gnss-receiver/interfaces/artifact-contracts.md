---
title: Artifact Contracts
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Artifact Contracts

Artifact contracts define the in-memory result of a receiver run before
infrastructure persists or indexes it.

## Owned Artifact Surfaces

- `RunArtifacts`
- acquisition result and explain collections
- tracking results and channel-state reports
- observation decisions, epochs, residuals, and quality reports
- navigation epochs and support-matrix capture

## Caller Expectations

- artifact records should reflect runtime meaning, not repository transport
- downstream crates may persist them, but that does not make persistence policy
  part of this contract
- receiver-boundary validation may consume these artifacts without owning
  repository inspection

## Closest Proof

- `crates/bijux-gnss-receiver/src/artifacts.rs`
- `crates/bijux-gnss-receiver/docs/ARTIFACTS.md`
