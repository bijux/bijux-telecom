# Codebase Rules

## Path Depth
- Max path depth: 4 from crate root. Example: `src/estimation/ekf/filter.rs` is OK, `src/estimation/ekf/models/helpers/foo.rs` is not.

## File Naming
- Snake case only.
- No numbered files (e.g., `foo_1.rs`, `ppp_impl_2.rs` are forbidden).
- Prefer semantic names: `state.rs`, `models.rs`, `filter.rs`.

## Module Ownership
- `bijux-gnss-core`: time, ids, observables schema, shared primitives.
- `bijux-gnss-signal`: DSP primitives (NCO, mixing, correlators, FFT helpers, code generators).
- `bijux-gnss-receiver`: pipeline orchestration (ingest → acquire → track → obs). No nav math.
- `bijux-gnss-nav`: ephemeris, satellite state, corrections, solvers, filters (EKF/PPP/RTK nav-side).
- `bijux-gnss-cli`: wiring + UX only. No domain logic.

## No Thin Modules
- A module must own types or logic.
- Modules that only re-export are forbidden.
- If a module only `pub use`s, inline exports in parent and remove it.

## Public API Surface
- `lib.rs` only exports top-level modules and key types.
- Internals stay private unless explicitly required.

## Tests
- Tests live under `tests/` unless they are tightly coupled to a module and add value as unit tests.


## Test Taxonomy

Use these prefixes for tests:

- `unit_*` for module-local unit tests.
- `prop_*` for property tests.
- `golden_*` for golden-file tests.
- `integration_*` for integration tests.
- `fault_*` for fault injection tests.
- `bench_*` for benchmarks.
