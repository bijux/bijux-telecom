---
title: Determinism And Purity
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Determinism And Purity

This page absorbs the old root-level determinism and purity guidance into the
runtime owner that most directly depends on those constraints.

## Main Expectations

- deterministic mode should preserve stable ordering and reproducible evidence
  on the same platform
- pure runtime zones should avoid environment access, wall-clock dependence,
  random behavior, and direct unmanaged I/O
- runtime-side regression proof should state clearly when cross-platform
  numerical tolerance rather than byte-identical output is the real contract

## Boundary Rule

The receiver owns the staged execution path where purity and determinism
pressure becomes operationally meaningful. Shared record meaning may still live
in core, but runtime discipline belongs here.

## First Proof Check

Inspect `crates/bijux-gnss-receiver/docs/TESTS.md`,
`crates/bijux-gnss-receiver/docs/RUNTIME.md`, and the deterministic or
reproducibility-oriented runtime tests before changing purity claims. Those
proofs show whether determinism is still being defended at the staged execution
boundary.
