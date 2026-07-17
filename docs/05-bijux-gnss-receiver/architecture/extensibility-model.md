---
title: Extensibility Model
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Extensibility Model

Extending `bijux-gnss-receiver` should mean adding or deepening a real runtime
family, not bolting convenience glue onto the public surface.

## Legitimate Extension Paths

- add a new runtime configuration or diagnostics surface inside `engine/`
- deepen an existing stage family inside acquisition, tracking, observations,
  or receiver-owned navigation adapters
- add a new runtime seam for samples, artifacts, or timing
- add synthetic proof or validation helpers that exercise the receiver boundary

## Illegitimate Extension Paths

- adding command-specific wrappers directly to `api.rs`
- adding repository file-layout policy into runtime artifacts
- re-exporting lower-level science only because one caller wants a shorter path

## Review Question

Can the new surface be named by a durable runtime responsibility? If not, it
probably does not belong here yet.
