# Architecture

`bijux-gnss-testkit` is the shared truth and fixture crate for GNSS testing.

## Source map

- `src/fixtures.rs` loads deterministic TOML and JSON fixtures and dataset entries.
- `src/antenna/` owns antenna-effect and synthesis helpers for test scenarios.
- `src/geometry.rs` owns geometry helpers needed by test support code.
- `src/position_truth/` owns observation synthesis, residual modeling, and scenario catalogs for
  position-related truth generation.
- `src/reference_data/` owns checked-in public truth datasets such as station truth and
  troposphere-elevation references.
- `src/reference_models/` owns independent model implementations used by the testkit to avoid
  circular validation against production helpers.
- `src/signal/` owns deterministic acquisition and signal-synthesis helpers used by tests.

## Dependency direction

This crate is allowed to depend on product crates where needed to build integration fixtures, but
it should keep the truth-producing logic independent enough that tests are not merely replaying the
same implementation paths they claim to verify.

## Test map

- `tests/scientific_independence.rs` protects the independence of test truth from specific `nav`
  helper implementations.
- `tests/integration_guardrails.rs` keeps the crate aligned with workspace guardrails.

## Design constraints

- truth helpers should prefer reference models and explicit formulas over calling convenience helpers
  from the production pipeline
- fixtures should stay deterministic and portable across crates
- if a helper is only meaningful to one test file and has no shared truth value, it probably does
  not belong in the shared testkit
