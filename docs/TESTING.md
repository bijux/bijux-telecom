# Testing Guide

This repository uses a test taxonomy to keep the suite navigable and deterministic.

## Test Types

- `unit_*`: small unit tests colocated with modules (`#[cfg(test)] mod tests`).
- `prop_*`: property tests (typically proptest) under `tests/`.
- `golden_*`: golden-file tests under `tests/`.
- `integration_*`: integration tests under `tests/`.
- `fault_*`: fault injection and adversarial tests under `tests/`.
- `bench_*`: benchmarks under `benches/`.

## Running Tests

- All tests: `cargo test`
- Unit tests only: `cargo test --lib`
- Integration/golden/fault tests: `cargo test --tests`
- Property tests (default): `cargo test prop_`
- Nextest: `cargo nextest run --config-file configs/rust/nextest.toml`

## Fuzzing

- Crate-local fuzz packages live under `crates/<owner>/fuzz/` rather than as peer crates in `crates/`.
- Build a fuzz package test surface with `cargo test --manifest-path crates/<owner>/fuzz/Cargo.toml --no-run`.
- Run a harness with `cargo fuzz run <target>` from the corresponding `crates/<owner>/fuzz/` directory when `cargo-fuzz` is installed.
- Keep fuzz targets adjacent to the crate that owns the parsed format, protocol, or configuration surface.

## Determinism Rules

- All randomized tests must accept a seed from configuration.
- Synthetic scenarios are deterministic by default.
- Property tests should have bounded ranges and include regression seeds when needed.

## Adding New Tests

- Place module-level unit tests inline in the module.
- Place integration or golden tests under `crates/<crate>/tests/`.
- Follow naming conventions so tests are discoverable.
