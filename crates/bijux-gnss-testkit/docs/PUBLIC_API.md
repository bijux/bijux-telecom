# Public API

`bijux-gnss-testkit` exposes a direct public module surface from `lib.rs`.

## Public modules

- `antenna` for antenna-effect truth and synthesis helpers
- `fixtures` for typed fixture and dataset loading
- `geometry` for reusable geometric helpers in tests
- `position_truth` for synthetic truth scenario construction
- `reference_data` for checked-in public truth assets and derived records
- `signal` for deterministic acquisition and signal synthesis helpers

## Internal-only modules

- `reference_models` is intentionally private even though it is central to the crate. It exists to
  support truth generation without turning its implementation details into a public dependency
  surface for other crates.

## Extension rule

Expose a helper publicly only when multiple crates or test families genuinely need it. Keep narrow
reference-model internals private by default.
