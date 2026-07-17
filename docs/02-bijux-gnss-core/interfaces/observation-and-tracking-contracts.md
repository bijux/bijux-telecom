---
title: Observation and Tracking Contracts
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Observation and Tracking Contracts

Observation and tracking records are where several crates meet. Signal code,
receiver runtime, navigation interpretation, and artifact persistence all need
the same language here.

## Main Record Families

- acquisition requests, results, evidence, refinements, and uncertainties
- tracking epochs, transitions, assumptions, and uncertainties
- observation epochs, manifests, decisions, support classes, and uncertainty
  classes
- differencing records and observation-quality metadata

## Why This Surface Is Sensitive

These records sit at the boundary between implemented behavior and shared
meaning. If they become too solver-specific, runtime-specific, or
persistence-specific, the rest of the repository loses a stable seam.

## Protecting Proof

- `crates/bijux-gnss-core/src/observation/`
- `crates/bijux-gnss-core/src/observation_quality.rs`
- `crates/bijux-gnss-core/docs/CONTRACTS.md`
- `crates/bijux-gnss-core/tests/tracking_artifact_validation.rs`
