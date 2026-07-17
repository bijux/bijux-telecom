---
title: Artifact Inspection Contracts
audience: mixed
type: explanation
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Artifact Inspection Contracts

Artifact inspection contracts define how persisted artifacts are interrogated
after they already exist.

## Main Owned Entry Points

- `artifact_explain`
- `artifact_validate`
- `ArtifactExplainResult`
- `ArtifactValidationResult`

## Why They Matter

This is where repository-facing review of outputs happens without pulling
callers back into runtime execution code.

## Boundary Rule

Infra owns post-run interpretation and validation entrypoints over persisted
artifacts. It does not own the original production of those artifacts.

## Protecting Proof

- `crates/bijux-gnss-infra/src/artifact_inspection/`
- `crates/bijux-gnss-infra/docs/VALIDATION.md`
