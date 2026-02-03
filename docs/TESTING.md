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
- Nextest: `cargo nextest run`

## Determinism Rules

- All randomized tests must accept a seed from configuration.
- Synthetic scenarios are deterministic by default.
- Property tests should have bounded ranges and include regression seeds when needed.

## Adding New Tests

- Place module-level unit tests inline in the module.
- Place integration or golden tests under `crates/<crate>/tests/`.
- Follow naming conventions so tests are discoverable.
