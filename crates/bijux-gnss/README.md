# bijux-gnss

## What this crate does
`bijux-gnss` is the user-facing crate for the GNSS workspace. It owns the `bijux` binary, parses
operator commands, prepares runtime context, renders reports, and delegates durable signal,
receiver, infrastructure, and navigation behavior to the lower-level crates that actually own it.

## Why this crate exists
The workspace needs one operator boundary that can expose GNSS workflows without leaking internal
crate structure into every command. This crate is that boundary.

## Public entrypoints

- `src/main.rs` owns the binary command surface
- `src/lib.rs` exposes curated re-exports of `core`, `receiver`, `signal`, and optional `nav`

## Ownership boundary
This crate owns command-line workflow assembly and reporting. It must not become the home for
low-level DSP, navigation science, repository run-layout rules, or receiver stage internals. The
boundary is documented in [docs/BOUNDARY.md](docs/BOUNDARY.md).

## Source layout

- `cli/command_catalog/` owns command names and argument families
- `cli/commands/` owns workflow handlers
- `cli/command_runtime/` owns runtime setup and reporting support
- `cli/command_support/` owns focused helper adapters used by commands

The architecture and test layout are documented in [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

## Documentation map
This crate keeps one root README and crate-specific docs under `docs/`:
- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)
- [docs/BOUNDARY.md](docs/BOUNDARY.md)

Repository work is governed by [../../README.md](../../README.md).
