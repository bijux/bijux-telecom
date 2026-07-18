# Workflows

`bijux-gnss-dev` owns repository-maintenance workflows that are important enough to encode as typed
commands rather than leave as ad hoc shell fragments.

## Audit allowlist workflow

`audit-allowlist` reads `audit-allowlist.toml` from the repository root and validates that every
security exception remains explicit, attributable, and time-bounded. This workflow protects the
repository from silent, permanent audit bypasses.

## Deny deviation workflow

`deny-policy-deviations` reads `configs/rust/deny.deviations.toml` and checks that local policy
deviations still have an owner, a reason, a review link, and an expiry. This keeps downstream
exceptions tied back to intentional standards work.

## Audit ignore argument workflow

`audit-ignore-args` converts the reviewed allowlist into `cargo audit --ignore ...` arguments. The
important contract is not string formatting; it is that automation derives its ignores from one
reviewed source instead of from duplicated CI configuration.

## Benchmark comparison workflow

`bench-compare` is the only workflow here that intentionally performs heavier execution:

- it runs a curated benchmark set for `bijux-gnss-receiver` and `bijux-gnss-nav`
- it writes raw benchmark output to `artifacts/benchmarks.txt`
- it writes the current normalized snapshot to `benchmarks/bencher_current.txt`
- it compares that snapshot with `benchmarks/bencher_baseline.txt`

When `--strict` is enabled, regression findings become a failing maintenance gate.

The governed benchmark evidence files and change discipline are documented in
[benchmark guide](BENCHMARKS.md).

## Output discipline

This crate is allowed to read repository files and write repository-scoped evidence, but those
writes must remain explicit:

- long-lived benchmark baselines belong in `benchmarks/`
- current-run evidence belongs in `artifacts/`
- hidden writes outside governed repository locations are not acceptable
