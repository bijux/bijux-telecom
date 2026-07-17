# Architecture

`bijux-gnss-signal` is the reusable signal-processing library for the GNSS workspace.

## Source map

- `src/api.rs` is the curated public surface.
- `src/catalog.rs` owns signal lookup, wavelength conversion, and default-signal selection helpers.
- `src/codes/` is partitioned by signal family:
  - GPS L1 C/A, L2C, and L5
  - Galileo E1 and E5
  - BeiDou B1I, B2I, and D1 secondary-code helpers
  - GLONASS L1 code and navigation-symbol helpers
- `src/dsp/` owns reusable processing primitives:
  - `front_end` for FIR front-end response handling
  - `local_code`, `sample_timing`, and `signal` for code-phase and sampling helpers
  - `nco` and `tracking` for oscillator and loop primitives
  - `replica` for synthetic carrier/code generation
  - `quality` and `spectrum` for front-end metrics and PSD analysis
- `src/obs_validation.rs` checks observation compatibility at the signal layer.
- `src/raw_iq.rs` and `src/samples.rs` own sample metadata and storage conversions.
- `src/error.rs` owns signal-layer error types.

## Dependency direction

This crate depends on `bijux-gnss-core` for foundational contracts and on general numeric crates
for DSP math. Higher-level crates such as `receiver`, `nav`, `infra`, and `testkit` consume this
crate; this crate must not depend on them.

## Test map

The test surface is organized by durable signal and DSP behavior:
- integration tests for every major code family and registry
- long-duration continuity tests for carrier, code, NCO, and replica helpers
- spectrum and front-end tests for BPSK, CBOC, and filtered responses
- property tests for C/A code, NCO behavior, and observation validation
- `tests/support/` for independently generated reference data and helper fixtures

## Design constraints

- code-generation helpers must stay deterministic and referenceable
- DSP primitives should remain reusable outside a specific receiver pipeline
- if a helper requires receiver runtime state or navigation solver state, it belongs in another
  crate
