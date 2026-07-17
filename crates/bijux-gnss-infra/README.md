# bijux-gnss-infra

## What this crate does
`bijux-gnss-infra` owns repository-facing infrastructure for the GNSS workspace. It handles run
layout and manifests, dataset registry and raw-IQ metadata loading, artifact inspection,
configuration overrides and sweep expansion, validation-reference adapters, and a curated
infrastructure-friendly API over `receiver`, `core`, `signal`, and optional `nav` surfaces.

## Why this crate exists
The workspace needs one place for filesystem-facing and repository-facing mechanics that do not
belong in `receiver`, `nav`, or the CLI. This crate keeps that boundary explicit.

## Public entrypoint
The curated downstream surface is `bijux_gnss_infra::api`.

## Ownership boundary
This crate owns infrastructure concerns and repository layout mechanics. It must not absorb signal
processing, navigation estimation, or command-line UX policy. The boundary is documented in
[docs/BOUNDARY.md](docs/BOUNDARY.md).

## Source layout

- `artifact_inspection/` validates and explains persisted artifacts
- `datasets/` resolves dataset registry entries and raw-IQ metadata
- `run_layout/` owns run directories, manifests, and reports
- `overrides/`, `experiments.rs`, and `sweep.rs` own experiment-parameter mutation and expansion
- `validate_reference.rs` and selected API re-exports expose comparison and validation helpers

The architecture and test layout are documented in [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

## Documentation map
This crate keeps one root README and crate-specific docs under `docs/`:
- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)
- [docs/BOUNDARY.md](docs/BOUNDARY.md)

Repository work is governed by [../../README.md](../../README.md).
