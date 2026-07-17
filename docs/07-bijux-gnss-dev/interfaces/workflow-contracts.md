---
title: Workflow Contracts
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Workflow Contracts

The commands are small, but each one owns a durable repository workflow.

## Audit Workflow Contract

The audit commands guarantee that reviewed security exceptions remain explicit,
attributed, and time-bounded.

## Deviation Workflow Contract

The deny-policy workflow guarantees that reviewed downstream deviations remain
owned, linked, and time-bounded.

## Ignore-Argument Workflow Contract

The `audit-ignore-args` workflow guarantees that automation derives
`cargo audit --ignore` flags from the same reviewed allowlist that maintainers
validate, instead of copying exception state into CI-only strings.

## Benchmark Workflow Contract

The benchmark workflow guarantees that the repository can rerun a curated
benchmark set, write current evidence, and compare the results against the
checked-in baseline under a configurable threshold.

## Protecting Proof

- `crates/bijux-gnss-dev/src/main.rs`
- `crates/bijux-gnss-dev/docs/WORKFLOWS.md`
- `crates/bijux-gnss-dev/docs/CONTRACTS.md`
