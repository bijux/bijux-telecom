---
title: Persisted Artifact Contracts
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Persisted Artifact Contracts

This page absorbs the old root-level artifact guide into the owner that
actually defines on-disk GNSS evidence layout and interpretation.

## Main Contract Families

- versioned artifact headers
- run-manifest and footprint persistence
- JSON and JSONL payload expectations for repository evidence
- compatibility expectations between writer and reader sides of the repository

## Boundary Rule

`bijux-gnss-infra` owns persisted artifact shape and interpretation. Product
crates may emit in-memory artifacts or records, but this crate owns how those
records are represented once the repository persists them.
