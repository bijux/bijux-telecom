# bijux-gnss-dev

`bijux-gnss-dev` owns maintainer tooling for the GNSS repository.

## Scope

This crate owns:

- quality gates for `audit-allowlist.toml`
- quality gates for `configs/rust/deny.deviations.toml`
- derived `cargo audit --ignore ...` arguments from the reviewed allowlist
- benchmark comparison workflows and their repository-scoped evidence outputs

This crate does not own operator-facing product commands, GNSS science, receiver execution, or
general-purpose shell helpers with no durable repository owner.

## Public surface

The public surface is the `bijux-gnss-dev` binary command set. There is no `lib.rs`, and downstream
crates should not treat maintainer governance workflows as a reusable library API.

## Source map

- `src/main.rs` owns subcommand parsing, repository-file validation, benchmark execution, and
  baseline comparison.
- `tests/integration_guardrails.rs` verifies the crate still fits the workspace structure rules.

## Documentation

The crate root intentionally contains only this README. Durable crate documentation lives under
`docs/`:

- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)
- [docs/BENCHMARKS.md](docs/BENCHMARKS.md)
- [docs/BOUNDARY.md](docs/BOUNDARY.md)
- [docs/COMMANDS.md](docs/COMMANDS.md)
- [docs/CONTRACTS.md](docs/CONTRACTS.md)
- [docs/GOVERNANCE_FILES.md](docs/GOVERNANCE_FILES.md)
- [docs/PUBLIC_API.md](docs/PUBLIC_API.md)
- [docs/TESTS.md](docs/TESTS.md)
- [docs/WORKFLOWS.md](docs/WORKFLOWS.md)

## Verification

Run from the repository root when changing this crate:

```sh
cargo test -p bijux-gnss-dev --test integration_guardrails
cargo run -p bijux-gnss-dev -- audit-allowlist
cargo run -p bijux-gnss-dev -- deny-policy-deviations
```

Repository-wide expectations are documented in [../../README.md](../../README.md).
