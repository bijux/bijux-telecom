# bijux-gnss-core

`bijux-gnss-core` owns the shared contract, identity, timekeeping, geometry, diagnostics, and
artifact foundations for `bijux-telecom`.

## Scope

This crate owns:

- canonical identifiers for constellations, satellites, and signals
- physical units, coordinate systems, and time-system conversion contracts
- acquisition, tracking, observation, differencing, and navigation-solution records
- diagnostic taxonomies and shared error categories
- versioned artifact envelopes and payload validation rules

This crate does not own raw sample ingestion, filesystem layout, DSP execution, navigation
estimation strategy, receiver orchestration, or operator command workflows.

## Public surface

`bijux_gnss_core::api` is the deliberate downstream entrypoint. Implementation modules stay private
unless they are intentionally re-exported there. That discipline keeps higher-level crates coupled
to stable GNSS meaning instead of to internal file layout.

## Source map

- `src/artifact/` owns versioned artifact envelopes and payload validation.
- `src/config.rs`, `src/diagnostic/`, and `src/error.rs` own validation and failure-report
  semantics.
- `src/ids.rs`, `src/time.rs`, `src/units.rs`, and `src/geo.rs` own foundational scientific types.
- `src/observation/`, `src/observation_quality.rs`, and `src/nav_solution.rs` own exchanged
  receiver and navigation records.
- `src/conventions.rs`, `src/stats.rs`, and `src/support_matrix.rs` own shared semantic helpers
  that are still crate-foundational rather than runtime-specific.

## Documentation

The crate root intentionally contains only this README. Durable crate documentation lives under
`docs/`:

- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)
- [docs/BOUNDARY.md](docs/BOUNDARY.md)
- [docs/CHANGE_RULES.md](docs/CHANGE_RULES.md)
- [docs/CONTRACTS.md](docs/CONTRACTS.md)
- [docs/CONTRACT_MAP.md](docs/CONTRACT_MAP.md)
- [docs/DIAGNOSTICS.md](docs/DIAGNOSTICS.md)
- [docs/INVARIANTS.md](docs/INVARIANTS.md)
- [docs/PUBLIC_API.md](docs/PUBLIC_API.md)
- [docs/SERIALIZATION.md](docs/SERIALIZATION.md)
- [docs/SUPPORT_MATRIX.md](docs/SUPPORT_MATRIX.md)
- [docs/TESTS.md](docs/TESTS.md)

## Verification

Run from the repository root when changing this crate:

```sh
cargo test -p bijux-gnss-core --test public_api_guardrail
cargo test -p bijux-gnss-core --test nav_artifact_validation
cargo test -p bijux-gnss-core --test tracking_artifact_validation
cargo test -p bijux-gnss-core --test prop_timekeeping
```

Repository-wide expectations are documented in [../../README.md](../../README.md).
