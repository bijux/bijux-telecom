# bijux-gnss-signal

## What this crate does
`bijux-gnss-signal` owns reusable signal-processing primitives for the GNSS workspace. It provides
signal catalogs, spreading-code generators, replica synthesis, front-end and spectrum helpers,
tracking-loop primitives, raw-IQ metadata contracts, and observation-compatibility validation.

## Why this crate exists
Signal math and code-generation logic must be shared across acquisition, tracking, synthetic test
data, and validation flows without being reimplemented differently in every crate. This crate is
that shared signal-processing layer.

## Public entrypoint
The curated downstream surface is `bijux_gnss_signal::api`, which re-exports the stable catalog,
code, DSP, sample-conversion, and trait families.

## Ownership boundary
This crate owns signal and DSP primitives, not receiver orchestration or navigation estimation. It
must not grow into a pipeline crate or a generic numeric bucket. The boundary is documented in
[docs/BOUNDARY.md](docs/BOUNDARY.md).

## Source layout

- `src/catalog.rs` resolves signal definitions and wavelength helpers.
- `src/codes/` owns spreading-code and secondary-code generation by constellation family.
- `src/dsp/` owns front-end, replica, spectrum, timing, NCO, and tracking helpers.
- `src/obs_validation.rs` owns dual-frequency and inter-frequency observation validation helpers.
- `src/raw_iq.rs` and `src/samples.rs` own raw-IQ metadata and sample conversion contracts.

The module and test structure is documented in [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md).

## Documentation map
This crate keeps one root README and crate-specific docs under `docs/`:
- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)
- [docs/BOUNDARY.md](docs/BOUNDARY.md)

Repository work is governed by [../../README.md](../../README.md).
