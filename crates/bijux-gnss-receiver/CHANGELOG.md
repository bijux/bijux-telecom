# Changelog
<a id="top"></a>

All notable changes to **bijux-gnss-receiver** are documented in this file.
Workspace-wide release notes live in the
[Bijux GNSS changelog](../../CHANGELOG.md).
This package adheres to [Semantic Versioning](https://semver.org) and the
[Keep a Changelog](https://keepachangelog.com/en/1.0.0/) format.

---

## 0.1.0 - 2026-07-19

### Added

- Added validated receiver and pipeline configuration with explicit defaults,
  per-signal capability selection, navigation settings, diagnostic controls,
  and typed rejection of unsupported combinations.
- Implemented composable receiver execution around sample sources, clocks,
  artifact sinks, metrics, traces, allocation evidence, and deterministic
  runtime state rather than hidden process-global side effects.
- Added acquisition across signal catalogs with FFT and search strategies,
  calibrated thresholds, Doppler-rate search, uncertainty estimates,
  component hypotheses, and accepted-or-refused evidence.
- Added assisted and related-signal acquisition with bounded search windows,
  oscillator-bias estimation, secondary-code and data-symbol hypotheses,
  cross-band follow-up, and safe fallback to full search.
- Implemented channel tracking with adaptive DLL and PLL behavior, vector and
  common-frequency aiding, secondary-code and navigation-symbol synchronization,
  reacquisition, calibrated lock decisions, and numerical-stability reports.
- Added observation production with transmit-time and code-period ambiguity
  resolution, receiver-clock handling, carrier-phase arcs, cycle-slip fusion,
  covariance, code-carrier divergence, Hatch smoothing, and quality validation.
- Integrated optional navigation execution and filtering, including SPP, RTK,
  PPP, precise-product policy, integrity thresholds, stage handoff, and
  preservation of degraded or refused navigation outcomes.
- Defined `RunArtifacts`, stage reports, diagnostics, support matrices,
  validation reports, tracking transitions, acquisition explanations, metrics,
  traces, and optional navigation epochs as inspectable receiver evidence.
- Added deterministic synthetic signal and navigation scenarios, truth-aware
  acquisition and tracking reports, covariance-realism checks, and external
  reference comparison with explicit alignment and tolerance policy.
- Published receiver, engine, pipeline, port, simulation, and evidence
  contracts through a curated `api` module with feature gates for navigation,
  precise products, tracing, reference checks, and allocation audits.
