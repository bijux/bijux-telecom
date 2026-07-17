---
title: Workflow Contracts
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# Workflow Contracts

Workflow contracts define how the command crate assembles lower-level crates
into operator-facing flows.

## Owned Workflow Families

- ingest and artifact workflows
- synthetic and export workflows
- diagnostics workflows
- pipeline-execution workflows
- validation workflows

## Boundary Rule

These are orchestration contracts, not scientific ownership claims. The command
crate owns how workflows are selected and presented, not the deeper behavior of
receiver, infra, signal, or nav crates.

## Closest Proof

- `crates/bijux-gnss/src/cli/commands/`
- `crates/bijux-gnss/docs/WORKFLOWS.md`
- `crates/bijux-gnss/docs/EXECUTION.md`
