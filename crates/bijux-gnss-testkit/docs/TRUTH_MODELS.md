# Truth Models

`bijux-gnss-testkit` exists to compute expected behavior without collapsing back into the same
production helper stack under test.

## Truth-model responsibilities

The crate owns:

- private reference models under `src/reference_models/`
- scenario truth generation under `src/position_truth/`
- antenna-effect and signal-synthesis truth helpers used by higher-level tests

## Independence rule

Reference models should stay independent enough that a production bug can still be detected by the
tests that depend on this crate. If a helper becomes a thin wrapper around a production solver or
runtime stage, the testkit has stopped doing its job.

## Visibility rule

`reference_models` remains private on purpose. Other crates should consume truth helpers and
reference data, not bind themselves to the implementation details of how test truth is derived.
