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
locations. `bench-compare` is the only owned command that writes them.

## Owned Output Locations

- `artifacts/benchmarks.txt` for raw benchmark output
- `benchmarks/bencher_current.txt` for the normalized current snapshot
- comparison against `benchmarks/bencher_baseline.txt` when a checked-in
  baseline exists

## Contract Rule

These outputs are part of the workflow boundary. Reviewers should be able to
tell which file is raw run evidence, which file is the current normalized
snapshot, and which file is the checked-in baseline. They should also be able
to tell that audit and deviation commands remain read-only.

## Protecting Proof

- `crates/bijux-gnss-dev/src/main.rs`
- `crates/bijux-gnss-dev/docs/OUTPUTS.md`
- `crates/bijux-gnss-dev/docs/BENCHMARKS.md`
