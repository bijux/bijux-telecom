# Changelog
<a id="top"></a>

All notable changes to **bijux-gnss-testkit** are documented in this file.
Workspace-wide release notes live in the
[Bijux GNSS changelog](../../CHANGELOG.md).
This repository-only package follows the workspace
[Semantic Versioning](https://semver.org) and
[Keep a Changelog](https://keepachangelog.com/en/1.0.0/) conventions.

---

## 0.1.0 - Unreleased

### Added

- Added deterministic fixture loaders for shared TOML, JSON, and dataset-style
  records with typed parsing and stable ordering for cross-crate tests.
- Added curated station, reference-coordinate, troposphere-elevation, and PPP
  convergence evidence with explicit units, frames, epochs, provenance, and
  intended consumers.
- Added independent geodetic and local-frame geometry helpers for checking
  coordinate conversions without calling the production conversion path under
  test.
- Added deterministic positioning scenario catalogs with satellite truth,
  observation synthesis, ephemerides, residual models, biases, clocks, and
  declared expected solutions.
- Added independent GPS broadcast-orbit and clock reference calculations for
  navigation-product and propagation comparisons.
- Added independent atmospheric and antenna-effect reference models, including
  controlled synthesis used by correction, PPP, and RTK tests.
- Added independent RTK reference calculations for differencing, baseline, and
  ambiguity evidence without reusing the production estimator being tested.
- Added deterministic signal synthesis, acquisition phase expectations, wrapped
  error checks, and clipped quantized-IQ cases for signal and receiver tests.
- Added executable scientific-independence checks that reject known production
  helper reuse in truth-producing code while documenting the limits of static
  name-based enforcement.
- Published repository-only `antenna`, `fixtures`, `geometry`,
  `position_truth`, `reference_data`, and `signal` modules for shared test
  consumers while keeping reference-model implementation details private.
