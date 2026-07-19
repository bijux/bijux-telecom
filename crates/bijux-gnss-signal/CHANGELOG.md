# Changelog
<a id="top"></a>

All notable changes to **bijux-gnss-signal** are documented in this file.
Workspace-wide release notes live in the
[Bijux GNSS changelog](../../CHANGELOG.md).
This package adheres to [Semantic Versioning](https://semver.org) and the
[Keep a Changelog](https://keepachangelog.com/en/1.0.0/) format.

---

## 0.1.0 - 2026-07-19

### Added

- Added a typed signal catalog for GPS, Galileo, GLONASS, and BeiDou with
  carrier frequencies, code rates, code lengths, component roles, wavelengths,
  and secondary-code metadata.
- Implemented GPS L1 C/A, L2C, and L5 spreading codes with deterministic PRN
  assignments, chip generation, and component-specific behavior.
- Implemented Galileo E1 and E5, GLONASS L1, and BeiDou B1I, B2I, and D1 code
  families with checked tables, assignments, and secondary-code handling.
- Defined raw-IQ metadata, sample formats, quantization profiles, byte
  interpretation, and complex sample conversion needed to decode captures
  without inventing dataset provenance.
- Added reusable sample timing, code-position, carrier trajectory, local-code,
  replica, modulation, and noise-power primitives with explicit phase,
  wrapping, unit, and normalization conventions.
- Added front-end processing, numerically bounded NCO state, spectrum analysis,
  signal-quality measurements, and carrier and code Doppler scaling helpers.
- Implemented acquisition signal models and correlator-ready replicas,
  including data, pilot, secondary-code, linear-rate carrier, and CBOC-related
  component behavior.
- Added DLL, PLL, and subcarrier discriminators, loop filters, lock-quality
  statistics, threshold calibration, and hysteretic tracking adaptation
  primitives.
- Exposed `SignalSource`, `SampleSource`, `Correlator`, and `SampleSink` traits
  so receivers can integrate reusable signal mechanics without transferring
  runtime scheduling or persistence ownership into this crate.
- Published a curated `api` module and observation-compatibility validation,
  supported by reference vectors and invariant tests for code families, sample
  conversion, indexing, spectra, replicas, and tracking math.
