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
- `configs/rust/nextest-slow-roster.txt`
- `benchmarks/bencher_baseline.txt`

## Contract Rule

These files are not generic config bags. They are reviewed repository contracts
whose shape and meaning are part of the maintainer workflow surface.

## Boundary Rule

If a repository file affects maintainer behavior but is not important enough to
be named and documented here, the workflow probably does not belong in this
crate yet.
