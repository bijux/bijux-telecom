# Contracts

`bijux-gnss-testkit` owns test-support contracts rather than product runtime contracts.

## Fixture-loading contract

`fixtures.rs` owns typed loading for:
- reference TOML payloads
- dataset-registry entries
- JSON golden files

This keeps fixture access deterministic and typed across the workspace.

The fixture boundary and its ownership rules are described in [FIXTURES.md](FIXTURES.md).

## Reference-data contract

`reference_data/` owns checked-in public truth inputs and derived truth records such as station
truth and troposphere-elevation references. These are shared evidence sources, not crate-local test
scratch files.

## Truth-generation contract

`position_truth/`, `antenna/`, and `signal/` own deterministic truth generation helpers that
higher-level crates use to validate acquisition, observation, and navigation behavior.

The independence and visibility rules behind those helpers are described in
[TRUTH_MODELS.md](TRUTH_MODELS.md).

## Independence contract

The crate must preserve enough independence from product helpers that it can validate them
meaningfully. When a truth helper becomes a thin wrapper around the runtime helper under test, the
contract has been violated.
