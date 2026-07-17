# bijux-gnss-signal

`bijux-gnss-signal` owns reusable GNSS signal definitions and DSP primitives.

## Scope

This crate owns:

- signal catalogs and physical wavelength helpers
- spreading-code and secondary-code generation across supported constellations
- front-end, replica, spectrum, timing, NCO, and tracking-loop primitives
- raw-IQ metadata and sample-conversion contracts
- signal-layer observation compatibility validation

This crate does not own receiver orchestration, persisted run layout, navigation estimation, or
operator command behavior.

## Public surface

`bijux_gnss_signal::api` is the deliberate downstream surface. It exposes catalog, code, DSP,
sample, validation, and trait families in a reusable shape while keeping receiver-runtime policy
out of this crate.

## Source map

- `src/catalog.rs` owns signal lookup and wavelength helpers.
- `src/codes/` owns constellation-specific code families.
- `src/dsp/` owns reusable signal-processing primitives.
- `src/obs_validation.rs` owns signal-level observation compatibility checks.
- `src/raw_iq.rs` and `src/samples.rs` own raw-sample contracts and conversions.

## Documentation

The crate root intentionally contains only this README. Durable crate documentation lives under
`docs/`:

- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)
- [docs/BOUNDARY.md](docs/BOUNDARY.md)
- [docs/CATALOG.md](docs/CATALOG.md)
- [docs/CODE_FAMILIES.md](docs/CODE_FAMILIES.md)
- [docs/CONTRACTS.md](docs/CONTRACTS.md)
- [docs/DSP.md](docs/DSP.md)
- [docs/RAW_IQ.md](docs/RAW_IQ.md)
- [docs/SAMPLES.md](docs/SAMPLES.md)
- [docs/TRAITS.md](docs/TRAITS.md)
- [docs/PUBLIC_API.md](docs/PUBLIC_API.md)
- [docs/TESTS.md](docs/TESTS.md)
- [docs/VALIDATION.md](docs/VALIDATION.md)

## Verification

Run from the repository root when changing this crate:

```sh
cargo test -p bijux-gnss-signal --test integration_signal_component_registry
cargo test -p bijux-gnss-signal --test integration_signal_spectrum_cboc
cargo test -p bijux-gnss-signal --test prop_obs_epoch_validation
```

Repository-wide expectations are documented in [../../README.md](../../README.md).
