# Shared Rust Make Contract

`bijux-makes-rs` provides reusable Rust gate execution for Bijux Cargo
workspaces. It is loaded by `bijux-makes` when the repository enables the
`rust` component.

## Public Targets

- `format-rs`: apply `cargo fmt`.
- `fmt-rs`: verify `cargo fmt`.
- `lint-rs`: run workspace Clippy with warnings denied.
- `test-rs`: run the configured fast nextest lane.
- `test-slow-rs`: run the configured slow nextest lane.
- `test-all-rs`: run all tests, including ignored tests.
- `audit-rs`: run cargo-deny and cargo-audit.
- `coverage-rs`: generate nextest coverage, LCOV, and summary reports.
- `rustdoc-check`: build workspace Rust documentation with warnings denied.
- `test-all-frozen`, `lint-frozen`, and `audit-frozen`: launch the
  corresponding gate from a pinned commit.
- `doctor-rs`: verify required Rust inputs and tools.

The generic `format`, `fmt`, `lint`, `test`, `test-slow`, `test-all`, `audit`,
`security`, and `coverage` targets aggregate these Rust targets through the
common contract.

Complete Rust test gates run up to eight tests concurrently by default.
Repositories may set `NEXTEST_THREADS_ALL` when a documented resource limit
requires a different value. Test groups may still serialize tests that share
mutable repository state.

## Required Repository Inputs

- `Cargo.toml`
- `configs/rust/nextest.toml`
- `configs/rust/deny.toml`

`configs/rust/nextest-slow-roster.txt` is optional. Non-comment lines are exact
nextest test names assigned to the slow lane.

## Repository Hooks

Repositories retain domain policy through prerequisite variables:

```make
RUST_FMT_PREREQUISITES
RUST_LINT_PREREQUISITES
RUST_TEST_PREREQUISITES
RUST_TEST_SLOW_PREREQUISITES
RUST_TEST_ALL_PREREQUISITES
RUST_AUDIT_PREREQUISITES
RUST_COVERAGE_PREREQUISITES
RUSTDOC_PREREQUISITES
```

These hooks must name real Make targets. The shared executor does not provide
fallbacks for missing tools or configured targets.

## Artifact Contract

Rust output is contained under `artifacts/rust/` by default:

```text
artifacts/rust/
  audit/<run-id>/report.txt
  cargo/home/
  coverage/<run-id>/
  fmt/<run-id>/report.txt
  lint/<run-id>/report.txt
  nextest/config/
  target/
  test/<run-id>/
  tmp/
```

The executor rejects an artifact root outside the repository's `artifacts/`
boundary.
