# bijux-gnss-policies

`bijux-gnss-policies` owns executable structural policy for the GNSS repository.

## Scope

This crate owns:

- crate-local guardrail execution through `check(crate_root, config)`
- typed guardrail configuration for source-tree and public-surface rules
- workspace policy tests for dependency direction, import layering, and repository structure
- read-only purity reporting for maintainers

This crate does not own product runtime behavior, GNSS scientific semantics, or general repository
automation that is unrelated to architecture policy.

## Public surface

`bijux_gnss_policies::api` is the deliberate library surface. It exposes the guardrail runner,
configuration, and canonical error/result types without exposing internal rule-module layout.

## Source map

- `src/guardrails/` owns source-tree, API-surface, and content-policy checks.
- `src/api.rs` is the curated downstream entrypoint.
- `src/bin/purity_report.rs` owns read-only reporting over workspace crate purity characteristics.
- `tests/` owns repository-wide structural assertions and policy snapshots.

## Documentation

The crate root intentionally contains only this README. Durable crate documentation lives under
`docs/`:

- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)
- [docs/BOUNDARY.md](docs/BOUNDARY.md)
- [docs/CONTRACTS.md](docs/CONTRACTS.md)
- [docs/GUARDRAILS.md](docs/GUARDRAILS.md)
- [docs/REPORTING.md](docs/REPORTING.md)
- [docs/SNAPSHOTS.md](docs/SNAPSHOTS.md)
- [docs/PUBLIC_API.md](docs/PUBLIC_API.md)
- [docs/TESTS.md](docs/TESTS.md)

## Verification

Run from the repository root when changing this crate:

```sh
cargo test -p bijux-gnss-policies --test integration_dep_rules
cargo test -p bijux-gnss-policies --test integration_workspace
cargo test -p bijux-gnss-policies --test integration_policy_snapshot
```

Repository-wide expectations are documented in [../../README.md](../../README.md).
