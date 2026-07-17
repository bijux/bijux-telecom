# bijux-gnss-policies

## What this crate does
`bijux-gnss-policies` owns the shared guardrail checks that keep the GNSS workspace structurally
coherent. It validates dependency direction, public API discipline, source-tree shape, and policy
conformance across the repository’s crates.

## Why this crate exists
Architecture standards decay quickly when they live only in conversations. This crate turns the
workspace’s structural expectations into code that can run in tests and in maintainer workflows.

## Public entrypoint
The curated downstream surface is `bijux_gnss_policies::api`, which exposes the guardrail runner,
configuration, and error/result types.

## Ownership boundary
This crate owns repository policy checks. It does not own product behavior, runtime plumbing, or
scientific semantics. The crate boundary is documented in [docs/BOUNDARY.md](docs/BOUNDARY.md).

## Source layout

- `src/guardrails/` contains the actual policy checks and configuration.
- `src/api.rs` defines the stable downstream entrypoint.
- `src/bin/purity_report.rs` is a repository report binary over workspace crate structure.
- `tests/` contains workspace-level policy suites and snapshots.

The module and test layout is documented in [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

## Documentation map
This crate keeps one root README and crate-specific docs under `docs/`:
- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)
- [docs/BOUNDARY.md](docs/BOUNDARY.md)

Repository work is governed by [../../README.md](../../README.md).
