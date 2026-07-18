# bijux-gnss-nav

`bijux-gnss-nav` owns navigation-domain science: navigation-product parsing,
orbit propagation, clock products, atmospheric and antenna corrections,
position estimation, RTK, PPP, RAIM, residuals, uncertainty, and time-system
interpretation needed by navigation algorithms.

Start here when the question is about turning observations or external
navigation products into a navigation claim. Do not start here for raw-IQ
ingest, receiver scheduling, signal-code generation, or CLI presentation.

## Reader Route

| question | go next |
| --- | --- |
| Which external format or product is parsed? | [docs/FORMATS.md](docs/FORMATS.md), `src/formats.rs` |
| Which correction or model owns the science? | [docs/CORRECTIONS.md](docs/CORRECTIONS.md), [docs/MODELS.md](docs/MODELS.md) |
| Which orbit or clock contract applies? | [docs/ORBITS.md](docs/ORBITS.md), `src/orbits/` |
| How is a solution estimated or refused? | [docs/ESTIMATION.md](docs/ESTIMATION.md), `src/estimation/` |
| What changed in this package? | [CHANGELOG.md](CHANGELOG.md) |

## Owned Boundary

- navigation-product formats and parsed product records
- broadcast and precise orbit helpers
- correction models for atmosphere, bias, combinations, tides, and carrier
  effects
- position estimation, uncertainty, residual, RTK, PPP, and RAIM behavior
- navigation-specific time interpretation and rollover handling

This crate does not own raw-IQ ingest, signal-code production, receiver
tracking loops, persisted run layout, or operator command presentation.

```mermaid
flowchart TB
    product["navigation products"]
    obs["receiver observations"]
    corrections["corrections and models"]
    estimator["estimator"]
    solution["solution or refusal evidence"]

    product --> corrections
    obs --> estimator
    corrections --> estimator
    estimator --> solution
```

## Source Map

- `src/orbits/` owns satellite-state and ephemeris logic.
- `src/formats.rs` owns navigation and precise-product parsing families.
- `src/corrections/` owns atmosphere, bias, combinations, tides, and
  carrier-aware computations.
- `src/estimation/` owns SPP, PPP, RAIM, EKF, and RTK surfaces.
- `src/models/` owns environmental and antenna models.
- `src/time.rs` and `src/time/` own navigation-specific time behavior.

## Documentation Map

- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)
- [docs/BOUNDARY.md](docs/BOUNDARY.md)
- [docs/CORRECTIONS.md](docs/CORRECTIONS.md)
- [docs/CONTRACTS.md](docs/CONTRACTS.md)
- [docs/ESTIMATION.md](docs/ESTIMATION.md)
- [docs/FORMATS.md](docs/FORMATS.md)
- [docs/MODELS.md](docs/MODELS.md)
- [docs/ORBITS.md](docs/ORBITS.md)
- [docs/PUBLIC_API.md](docs/PUBLIC_API.md)
- [docs/TESTS.md](docs/TESTS.md)
- [docs/TIME.md](docs/TIME.md)

## Verification Focus

Use navigation tests that match the scientific surface changed:

```sh
cargo test -p bijux-gnss-nav --test integration_sp3
cargo test -p bijux-gnss-nav --test integration_position
cargo test -p bijux-gnss-nav --test integration_rtk_double_difference
```

Repository-wide lanes and package routing are documented in
[../../README.md](../../README.md).
