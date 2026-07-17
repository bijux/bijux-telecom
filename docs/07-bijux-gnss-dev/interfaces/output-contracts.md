---
title: Output Contracts
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Output Contracts

`bijux-gnss-dev` emits repository-maintenance evidence into named governed
locations.

## Owned Output Locations

- `artifacts/benchmarks.txt`
- `benchmarks/bencher_current.txt`
- comparison against `benchmarks/bencher_baseline.txt`

## Contract Rule

These outputs are part of the workflow boundary. Reviewers should be able to
tell which file is raw run evidence, which file is the current normalized
snapshot, and which file is the checked-in baseline.
