# Contract Map

This file maps the main `bijux-gnss-core` modules to the contract families they own.

## Module-to-contract ownership

- `src/artifact/`
  owns versioned artifact envelopes, headers, kind tagging, and payload validation traits
- `src/config.rs`
  owns schema versioning and configuration validation-report contracts
- `src/diagnostic/` and `src/error.rs`
  own shared failure taxonomies, severity, and event/summarization records
- `src/ids.rs`
  owns constellation, satellite, signal, and related identity contracts
- `src/time.rs`
  owns GPS, UTC, TAI, sample-time, and leap-second contracts
- `src/units.rs`
  owns strong physical unit wrappers and conversion semantics
- `src/geo.rs`
  owns geodetic and Cartesian coordinate contracts
- `src/observation/` and `src/observation_quality.rs`
  own acquisition, tracking, observation, differencing, and quality records
- `src/nav_solution.rs`
  owns navigation-solution, residual, refusal, and inter-system-bias records
- `src/support_matrix.rs`
  owns supported-signal inventory contracts

## Why this map exists

This crate is intentionally dense because it is foundational. The point of this file is to make
ownership visible before contributors add one more "shared" type to the wrong module or the wrong
crate.
