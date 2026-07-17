# bijux-gnss-receiver

`bijux-gnss-receiver` owns GNSS receiver runtime orchestration for the workspace.

## Scope

This crate owns:

- receiver configuration and runtime state
- acquisition, tracking, observation, and optional navigation stage execution
- clock, sample-source, and artifact-sink boundaries
- in-memory run artifacts and receiver-boundary validation helpers
- synthetic execution helpers exposed at the receiver boundary

This crate does not own repository persistence, operator workflow policy, low-level signal-code
generation, or standalone navigation science.

## Public surface

`bijux_gnss_receiver::api` is the deliberate downstream surface. It exposes the receiver boundary,
its stage engines, runtime configuration, and curated re-exports from lower layers without turning
the crate into a catch-all domain library.

## Source map

- `src/engine/` owns runtime configuration, logging, metrics, and receiver composition.
- `src/pipeline/` owns staged acquisition, tracking, observations, and optional navigation flows.
- `src/io/` and `src/ports/` own source/sink and clock abstractions.
- `src/artifacts.rs`, `src/validation_report.rs`, and related modules own runtime-side outputs.
- `src/sim/` owns synthetic receiver execution helpers.

## Documentation

The crate root intentionally contains only this README. Durable crate documentation lives under
`docs/`:

- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)
- [docs/ARTIFACTS.md](docs/ARTIFACTS.md)
- [docs/BOUNDARY.md](docs/BOUNDARY.md)
- [docs/CONTRACTS.md](docs/CONTRACTS.md)
- [docs/PIPELINE.md](docs/PIPELINE.md)
- [docs/PORTS.md](docs/PORTS.md)
- [docs/PUBLIC_API.md](docs/PUBLIC_API.md)
- [docs/REFERENCE_VALIDATION.md](docs/REFERENCE_VALIDATION.md)
- [docs/RUNTIME.md](docs/RUNTIME.md)
- [docs/SIMULATION.md](docs/SIMULATION.md)
- [docs/TESTS.md](docs/TESTS.md)

## Verification

Run from the repository root when changing this crate:

```sh
cargo test -p bijux-gnss-receiver --test integration_basic
cargo test -p bijux-gnss-receiver --test integration_receiver_support_matrix_inventory
cargo test -p bijux-gnss-receiver --test integration_navigation_pvt_accuracy_budget
```

Repository-wide expectations are documented in [../../README.md](../../README.md).
