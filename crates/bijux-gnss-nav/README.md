# bijux-gnss-nav

`bijux-gnss-nav` owns navigation-domain GNSS science for the workspace.

## Scope

This crate owns:

- broadcast and precise orbit state plus ephemeris interpretation
- navigation-message and reference-product parsing
- atmospheric, bias, and signal-combination corrections
- position, integrity, PPP, and RTK estimation behavior
- supporting physical models and navigation-time helpers

This crate does not own receiver scheduling, repository run layout, dataset registry mechanics, or
operator command workflows.

## Public surface

`bijux_gnss_nav::api` is the deliberate downstream surface. It organizes exports by durable
scientific role rather than by internal file layout so higher-level crates consume navigation
capabilities without coupling themselves to parser and solver internals.

## Source map

- `src/orbits/` owns satellite-state and ephemeris logic.
- `src/formats/` owns navigation and precise-product parsing families.
- `src/corrections/` owns atmosphere, bias, and combination computations.
- `src/estimation/` owns SPP, PPP, RAIM, EKF, and RTK surfaces.
- `src/models/` owns supporting physical models.
- `src/time/` owns navigation-specific time utilities.

## Documentation

The crate root intentionally contains only this README. Durable crate documentation lives under
`docs/`:

- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)
- [docs/BOUNDARY.md](docs/BOUNDARY.md)
- [docs/CONTRACTS.md](docs/CONTRACTS.md)
- [docs/ESTIMATION.md](docs/ESTIMATION.md)
- [docs/FORMATS.md](docs/FORMATS.md)
- [docs/PUBLIC_API.md](docs/PUBLIC_API.md)
- [docs/TESTS.md](docs/TESTS.md)

## Verification

Run from the repository root when changing this crate:

```sh
cargo test -p bijux-gnss-nav --test integration_position
cargo test -p bijux-gnss-nav --test integration_precise_products
cargo test -p bijux-gnss-nav --test integration_rtk_baseline_accuracy
```

Repository-wide expectations are documented in [../../README.md](../../README.md).
