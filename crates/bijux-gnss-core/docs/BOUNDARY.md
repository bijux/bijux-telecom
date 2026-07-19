# Boundary

Owner: shared GNSS contracts and physical foundations

## Scope

`bijux-gnss-core` owns durable cross-crate meaning:
- identity types for constellations, satellites, and signals
- physical units and coordinate/time conversions
- observation and navigation-solution records
- diagnostics, canonical errors, and configuration validation records
- versioned artifact envelopes and payload validators

## What this crate must own

- types that multiple higher-level crates need to exchange without reinterpretation
- validation rules that travel with those shared contracts
- pure scientific and geometric helpers that define shared meaning rather than runtime policy
- stable artifact payload shapes consumed by inspection, export, and validation flows

## What this crate must not own

- raw sample ingestion or filesystem IO
- signal generation, correlators, or DSP execution
- orbit propagation, atmospheric estimation, or navigation solvers
- receiver orchestration, runtime scheduling, or artifact persistence
- CLI parsing, reporting, or operator workflow policy

## Allowed dependencies

- general-purpose libraries needed for serde, numeric types, and error handling
- no dependencies on higher-level workspace crates

## Effect model

This crate should be effectively pure from a product perspective. It may allocate, serialize, and
validate in-memory data, but it should not own repository layout decisions, dataset discovery, file
movement, or process orchestration.

## Change standard

Any change here is amplified across the workspace. Rename, remove, or reshape contracts only when
the new boundary is clearer and the downstream impact is deliberate. If a concept is transitional,
local, or workflow-specific, keep it out of `bijux-gnss-core`.
