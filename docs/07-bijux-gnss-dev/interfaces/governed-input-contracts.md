---
title: Governed Input Contracts
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Governed Input Contracts

This binary consumes a narrow set of reviewed repository inputs.

## Governed Inputs

- `audit-allowlist.toml`
- `configs/rust/deny.deviations.toml`
- `benchmarks/bencher_baseline.txt`

## Guarded But Not Command-Consumed

- `configs/rust/nextest-slow-roster.txt`

The slow roster matters to this handbook, but it is guarded through
`tests/integration_nextest_suite_selection.rs` rather than being read by a
maintainer command in `src/main.rs`.

## Contract Rule

These files are not generic config bags. They are reviewed repository contracts
whose shape and meaning are part of the maintainer workflow surface.

## Boundary Rule

If a repository file affects maintainer behavior but is not important enough to
be named and documented here, the workflow probably does not belong in this
crate yet.

## Protecting Proof

- `crates/bijux-gnss-dev/src/main.rs`
- `crates/bijux-gnss-dev/docs/GOVERNANCE_FILES.md`
- `crates/bijux-gnss-dev/tests/integration_nextest_suite_selection.rs`
