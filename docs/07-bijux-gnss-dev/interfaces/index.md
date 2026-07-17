---
title: Interfaces
audience: mixed
type: index
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Interfaces

Open this section when the question is what `bijux-gnss-dev` publicly promises
to maintainers and repository automation.

## Read These First

- open [Command Surface](command-surface.md) first when the question is which
  maintainer commands exist
- open [Governed Input Contracts](governed-input-contracts.md) when the question
  is what repository files this binary treats as reviewed inputs
- open [Output Contracts](output-contracts.md) when the question is where
  maintenance evidence is emitted

## Pages In This Section

- [Command Surface](command-surface.md)
- [Governed Input Contracts](governed-input-contracts.md)
- [Output Contracts](output-contracts.md)
- [Stability Commitments](stability-commitments.md)
- [Binary Boundary](binary-boundary.md)
- [Command Entry Contracts](command-entry-contracts.md)
- [Workflow Contracts](workflow-contracts.md)
- [Entry Points And Examples](entry-points-and-examples.md)
- [Compatibility Commitments](compatibility-commitments.md)

## First Public Surfaces

- `crates/bijux-gnss-dev/src/main.rs`
- `crates/bijux-gnss-dev/docs/COMMANDS.md`
- `crates/bijux-gnss-dev/docs/GOVERNANCE_FILES.md`
- `crates/bijux-gnss-dev/docs/OUTPUTS.md`

## Leave This Section When

- leave for [Architecture](../architecture/) when the question is about code
  organization rather than maintainer contract
- leave for [Operations](../operations/) when the interface is clear and the
  next question is how to change it safely
