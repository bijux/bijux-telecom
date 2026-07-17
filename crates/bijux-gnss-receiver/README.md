# bijux-gnss-receiver

## What this crate does
`bijux-gnss-receiver` owns GNSS receiver-pipeline orchestration. It takes signal sources and
receiver configuration, runs acquisition/tracking/observation/navigation stages, emits run
artifacts, and exposes simulation and validation support behind the receiver boundary.

## Why this crate exists
Signal primitives and navigation science are not enough by themselves; the workspace also needs the
runtime that connects them into a receiver. This crate is that integration boundary.

## Public entrypoint
The curated downstream surface is `bijux_gnss_receiver::api`.

## Ownership boundary
This crate owns receiver runtime composition and stage execution. It must not own repository run
layouts, operator CLI workflows, or duplicate the low-level science already owned by `signal` and
`nav`. The boundary is documented in [docs/BOUNDARY.md](docs/BOUNDARY.md).

## Source layout

- `engine/` owns runtime configuration, logging, metrics, and the top-level receiver composition
- `pipeline/` owns acquisition, tracking, observations, navigation, and related stage helpers
- `io/` and `ports/` own source/sink boundaries and clock abstractions
- `sim/` and validation modules own synthetic execution and validation-report support

The architecture and verification map are documented in [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

## Documentation map
This crate keeps one root README and crate-specific docs under `docs/`:
- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)
- [docs/BOUNDARY.md](docs/BOUNDARY.md)

Repository work is governed by [../../README.md](../../README.md).
