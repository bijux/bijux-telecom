# bijux-gnss-testkit

## What this crate does
`bijux-gnss-testkit` owns deterministic test support for the GNSS workspace: reference data,
reference models, scenario truth generation, antenna-effect synthesis, and reusable signal/test
fixtures that higher-level crates can consume without inventing their own physics shortcuts.

## Why this crate exists
The workspace needs one place where test truth is curated deliberately. Without a dedicated testkit,
`nav`, `receiver`, and CLI tests would keep copying fragile helpers and would start validating code
with the same code under test.

## Public entrypoints

- `antenna` for antenna-effect truth helpers
- `fixtures` for deterministic fixture loading
- `geometry` for geometric support shared by tests
- `position_truth` for synthetic truth scenarios and residual models
- `reference_data` for checked-in public reference datasets
- `signal` for deterministic acquisition and signal-synthesis helpers

## Ownership boundary
This crate owns truth generation and test support. It must not become a second implementation of
the runtime algorithms it is supposed to validate. The boundary is documented in
[docs/BOUNDARY.md](docs/BOUNDARY.md).

## Source layout

- `src/reference_models/` contains independent models used to keep truth generation separate from
  runtime helpers.
- `src/reference_data/` contains public checked-in truth inputs and derivations.
- `src/position_truth/` contains synthetic scenario truth construction.

The architecture and test layout are documented in [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

## Documentation map
This crate keeps one root README and crate-specific docs under `docs/`:
- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)
- [docs/BOUNDARY.md](docs/BOUNDARY.md)

Repository work is governed by [../../README.md](../../README.md).
