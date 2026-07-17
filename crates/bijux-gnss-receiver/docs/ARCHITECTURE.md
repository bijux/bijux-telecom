# Architecture

`bijux-gnss-receiver` is the runtime receiver crate for the workspace.

## Source map

- `src/api.rs` is the curated downstream surface.
- `src/engine/` owns receiver configuration, runtime state, logging, support-matrix handling, and
  top-level receiver composition.
- `src/pipeline/` owns staged execution for acquisition, tracking, observation construction,
  navigation, navigation filtering, and related differencing or assistance helpers.
- `src/io/` and `src/ports/` own data-source, sample-source, artifact-sink, and clock boundaries.
- `src/artifacts.rs` and validation-report modules own emitted runtime artifact support.
- `src/reference_validation.rs` bridges receiver outputs to reference-comparison flows.
- `src/sim/` owns synthetic scenario execution and canonical synthetic helpers used by tests and
  validation.
- `src/covariance_realism.rs` and `src/validation_report.rs` own post-solution realism and
  validation reporting when the `nav` feature is enabled.

## Dependency direction

This crate builds on `core`, `signal`, and optionally `nav`. Higher-level crates such as `infra`
and the CLI depend on the receiver boundary rather than reaching into stage internals directly.

## Test map

The receiver test surface is intentionally large:
- acquisition behavior, accuracy budget, and explainability tests
- tracking dynamics, lock, continuity, and truth-table tests
- observation quality, smoothing, and uncertainty tests
- navigation, protection-level, multipath, and validation-report tests
- receiver support-matrix, feature-boundary, determinism, and artifact tests
- property tests and shared support fixtures under `tests/support/`

## Design constraints

- stage-specific math should stay in the stage modules instead of spreading through the runtime
- receiver composition should call into `signal` and `nav`, not reimplement their owned science
- if a helper is repository-facing rather than runtime-facing, it belongs in `infra`
