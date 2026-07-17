# Outputs

`bijux-gnss-dev` writes evidence outputs for repository maintenance workflows.

## Owned output locations

The crate currently writes or maintains:

- `artifacts/benchmarks.txt`
- `benchmarks/bencher_current.txt`
- `benchmarks/bencher_baseline.txt`

## Why this is documented

Maintainer tooling should not scatter outputs unpredictably. Benchmark evidence needs stable,
discoverable locations so reviewers can understand what is baseline, what is current-run evidence,
and what is ephemeral.

## Boundary rule

This crate may emit repository-governed maintenance evidence. It should not quietly create ad hoc
output locations unrelated to its owned workflows.
