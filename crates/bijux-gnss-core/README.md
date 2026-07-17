# bijux-gnss-core

## What this crate does
`bijux-gnss-core` owns the stable scientific and engineering contracts shared by the GNSS
workspace. It defines typed identities, time systems, units, observations, navigation-solution
records, diagnostics, configuration validation, and versioned artifact payloads without owning
runtime execution.

## Why it exists
The rest of the workspace needs one place where foundational GNSS meaning is defined once. This
crate keeps those definitions durable so `signal`, `nav`, `receiver`, `infra`, `testkit`, and the
CLI do not drift into incompatible copies of the same concepts.

## Public entrypoint
The curated downstream surface is `src/api.rs`, exported as `bijux_gnss_core::api`. Internal
modules stay crate-private unless they are intentionally re-exported there.

## Ownership boundaries
This crate owns data contracts, validation rules, diagnostics, and pure scientific helpers. It must
not own sample IO, receiver orchestration, navigation solvers, filesystem layouts, or command-line
behavior. The crate boundary is documented in [docs/BOUNDARY.md](docs/BOUNDARY.md).

## Source layout
The crate is organized around a few durable domains:
- `artifact` for versioned artifact envelopes and payload validators
- `config` and `diagnostic` for configuration and structured failure reporting
- `ids`, `time`, `units`, and `geo` for shared physical foundations
- `observation` and `nav_solution` for receiver-facing and navigation-facing records

The module map and test ownership are documented in [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

## Verification
The most important checks for this crate are public-surface guardrails, property tests for time and
units, and artifact validation coverage under `tests/`.

## Documentation map
This crate keeps one root README and crate-specific docs under `docs/`:
- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)
- [docs/BOUNDARY.md](docs/BOUNDARY.md)

Repository work is governed by [../../README.md](../../README.md).
