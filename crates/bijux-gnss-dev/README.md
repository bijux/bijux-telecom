# bijux-gnss-dev

## What this crate does
`bijux-gnss-dev` owns maintainer-only tooling for the GNSS workspace. It validates audit
allowlists, checks deny-policy governance files, derives `cargo audit` ignore arguments from the
 reviewed allowlist, and runs benchmark comparison workflows that write evidence into repository
artifacts.

## Why this crate exists
The workspace needs operational tooling that is too repository-specific to belong in product crates
and too important to leave as ad hoc shell fragments. This crate keeps that tooling typed,
testable, and versioned with the repository.

## Ownership boundary
This crate owns maintainer workflows, not GNSS science or pipeline behavior. It must not turn into
a second CLI for product features or a generic utility bucket. The boundary is documented in
[docs/BOUNDARY.md](docs/BOUNDARY.md).

## Source layout

- `src/main.rs` contains the full command surface and command implementations.
- `tests/integration_guardrails.rs` keeps the crate aligned with workspace guardrails.

The crate structure is documented in [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

## Documentation map
This crate keeps one root README and crate-specific docs under `docs/`:
- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)
- [docs/BOUNDARY.md](docs/BOUNDARY.md)

Repository work is governed by [../../README.md](../../README.md).
