# bijux-gnss-infra

`bijux-gnss-infra` owns repository-facing GNSS infrastructure.

## Scope

This crate owns:

- dataset registry records and raw-IQ metadata resolution
- run directory identity, reports, manifests, and history appends
- artifact inspection and validation over persisted outputs
- experiment sweep expansion, profile overrides, and provenance hashing
- infrastructure-friendly API composition over lower-level crates

This crate does not own DSP implementation, navigation estimation strategy, receiver stage
execution, or operator command policy.

## Public surface

`bijux_gnss_infra::api` is the deliberate downstream surface. It mixes infrastructure-owned helpers
with selected re-exports from lower layers so repository-facing callers can work through one
boundary without transferring product ownership into this crate.

## Source map

- `src/artifact_inspection/` owns post-run artifact explanation and validation.
- `src/datasets/` owns dataset registry parsing, coordinate parsing, and metadata resolution.
- `src/run_layout/` owns run identity, directories, manifests, reports, and history persistence.
- `src/overrides/`, `src/experiments.rs`, and `src/sweep.rs` own typed configuration mutation.
- `src/hash/` owns provenance-oriented hashing helpers.
- `src/validate_reference.rs` owns infrastructure-side validation adapters.

## Documentation

The crate root intentionally contains only this README. Durable crate documentation lives under
`docs/`:

- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)
- [docs/BOUNDARY.md](docs/BOUNDARY.md)
- [docs/CONTRACTS.md](docs/CONTRACTS.md)
- [docs/DATASETS.md](docs/DATASETS.md)
- [docs/PUBLIC_API.md](docs/PUBLIC_API.md)
- [docs/RUN_LAYOUT.md](docs/RUN_LAYOUT.md)
- [docs/TESTS.md](docs/TESTS.md)

## Verification

Run from the repository root when changing this crate:

```sh
cargo test -p bijux-gnss-infra --test integration_overrides
cargo test -p bijux-gnss-infra --test integration_guardrails
```

Repository-wide expectations are documented in [../../README.md](../../README.md).
