---
title: Stability Commitments
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Stability Commitments

This page replaces the old root-level API stability note. It explains which
repository-facing interfaces are expected to move slowly and which remain
experimental.

## Stable Surfaces

- shared artifact types and headers in `bijux-gnss-core`
- run-layout and manifest helpers in `bijux-gnss-infra`
- documented maintainer command inputs and outputs in `bijux-gnss-dev` once
  they are referenced by a governed make lane or reviewed repository workflow

## Experimental Surfaces

- navigation estimator families whose scientific scope is still expanding
- receiver pipeline stages and DSP-adjacent helpers that are not yet committed
  as durable public APIs
- CLI output schemas that are still evolving with operator workflows

## Maintainer Rule

Do not call a surface stable merely because it already has callers. Stability
means the repository is willing to pay the compatibility cost.

## Protecting Proof

Inspect `crates/bijux-gnss-dev/docs/COMMANDS.md`,
`crates/bijux-gnss-dev/docs/WORKFLOWS.md`,
`crates/bijux-gnss-dev/docs/OUTPUTS.md`, and
`crates/bijux-gnss-dev/src/main.rs`. Then inspect
`crates/bijux-gnss-dev/tests/integration_guardrails.rs` to confirm that the
stability claims here still match the actual maintainer command surface and
governed evidence contracts.
