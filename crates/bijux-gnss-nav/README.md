# bijux-gnss-nav

## What this crate does
`bijux-gnss-nav` owns the GNSS navigation domain for the workspace: orbit and ephemeris handling,
navigation-format parsing, atmospheric and bias corrections, estimation engines for positioning and
integrity, and navigation-time utilities shared by downstream crates.

## Why this crate exists
Navigation logic has enough depth and scientific cohesion to stand as its own crate. It is where
the workspace turns signal-level observations and reference products into satellite state,
corrections, and position solutions.

## Public entrypoint
The curated downstream surface is `bijux_gnss_nav::api`.

## Ownership boundary
This crate owns navigation-domain science and estimation logic. It must not own receiver runtime
orchestration, repository run layout, or operator command workflows. The boundary is documented in
[docs/BOUNDARY.md](docs/BOUNDARY.md).

## Source layout

- `orbits/` owns satellite-state and ephemeris logic
- `formats/` owns navigation and precise-product parsing/formatting
- `corrections/` owns atmosphere, bias, and signal-combination helpers
- `estimation/` owns SPP, EKF, PPP, RAIM, and RTK estimation flows
- `models/` owns supporting physical models
- `time/` owns navigation-specific time utilities

The architecture and test layout are documented in [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

## Documentation map
This crate keeps one root README and crate-specific docs under `docs/`:
- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)
- [docs/BOUNDARY.md](docs/BOUNDARY.md)

Repository work is governed by [../../README.md](../../README.md).
