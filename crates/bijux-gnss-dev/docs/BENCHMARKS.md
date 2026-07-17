# Benchmarks

`bijux-gnss-dev` does not own benchmark definitions, but it does own benchmark governance.

## What this crate governs

`bench-compare` currently owns:

- the curated benchmark package set
- normalized snapshot extraction from bencher output
- current-run evidence emission into `artifacts/`
- baseline comparison against `benchmarks/bencher_baseline.txt`
- strict-mode failure when regressions exceed the configured threshold

## What this crate does not own

- the benchmark code inside product crates
- receiver or navigation performance semantics themselves
- permanent storage policy outside the governed benchmark locations

## Change discipline

If the curated benchmark set changes, update this file and [WORKFLOWS.md](WORKFLOWS.md) in the same
change set so maintainers can see what the repository is actually gating.
