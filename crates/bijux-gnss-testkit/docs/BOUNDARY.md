# Boundary

Owner: shared test truth, fixtures, and independent reference models

## Scope

`bijux-gnss-testkit` owns:
- deterministic fixture loading
- checked-in reference datasets used by tests
- independent reference models used to compute expected behavior
- scenario and truth generation for acquisition, observation, antenna, and position tests

## What this crate must not own

- production receiver orchestration
- navigation solver implementations
- infrastructure run layouts or operator CLI behavior
- “test helpers” that simply call the same runtime helper the test is supposed to validate

## Independence rule

Where practical, this crate should compute truth through reference models or explicit formulas
instead of leaning on the same `nav` or `receiver` helper stack used in production. The scientific
independence test exists to keep that boundary honest.

## Effect model

This crate may read checked-in reference files and expose deterministic helper functions. It should
not own mutable repository workflows or runtime side effects outside test support.
