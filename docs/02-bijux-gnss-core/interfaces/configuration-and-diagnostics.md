---
title: Configuration and Diagnostics
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Configuration and Diagnostics

Configuration and diagnostics are contract surfaces because multiple crates
consume them, even though neither surface is the top-level product workflow.

## Configuration Surface

Core owns:

- `BijuxGnssConfig`
- `SchemaVersion`
- `ValidateConfig`
- `ValidationReport`

This is contract-level validation shape, not repository config discovery or CLI
config UX.

## Diagnostic Surface

Core owns:

- diagnostic codes and metadata
- structured diagnostic events
- summaries, summary entries, and severity levels
- canonical error family names

This lets downstream crates emit structured evidence using one vocabulary even
when their runtime behavior differs.

## Protecting Proof

- `crates/bijux-gnss-core/src/config.rs`
- `crates/bijux-gnss-core/src/diagnostic/`
- `crates/bijux-gnss-core/docs/DIAGNOSTICS.md`
