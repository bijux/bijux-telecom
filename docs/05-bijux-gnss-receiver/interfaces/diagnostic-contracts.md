---
title: Diagnostic Contracts
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Diagnostic Contracts

This page absorbs the old root-level diagnostics guide into the runtime owner
that actually emits and summarizes stage-facing issues.

## Main Diagnostic Guarantees

- pipeline stages emit stable diagnostic meaning rather than ad hoc log prose
- severity remains explicit
- receiver workflows may summarize diagnostics by run because runtime execution
  owns the operational context

## Boundary Rule

Shared diagnostic record shapes may live in core, but receiver-owned diagnostic
behavior and stage-facing meaning belong here with runtime orchestration.

## Protecting Proof

- `crates/bijux-gnss-receiver/docs/ARTIFACTS.md`
- `crates/bijux-gnss-receiver/docs/RUNTIME.md`
- `crates/bijux-gnss-receiver/tests/integration_tracking_channel_state_reports.rs`
