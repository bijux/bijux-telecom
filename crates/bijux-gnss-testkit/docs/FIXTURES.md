# Fixtures

`bijux-gnss-testkit` owns fixture access as a typed, shared test contract.

## Fixture responsibilities

`src/fixtures.rs` owns typed loading for:

- reference TOML payloads
- dataset-style entries used in tests
- JSON and other checked-in golden data used across crate boundaries

## Why this is centralized

Without a shared fixture boundary, every crate starts inventing slightly different file lookup,
parsing, and defaulting rules. That turns tests into a collection of local conventions instead of a
coherent verification surface.

## Boundary rules

- Checked-in test evidence may be loaded here.
- Repository-wide dataset discovery policy does not belong here.
- Production persistence and run layout do not belong here.
