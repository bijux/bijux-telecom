# Changelog
<a id="top"></a>

All notable changes to **bijux-gnss-nav** are documented in this file.
Workspace-wide release notes live in the
[Bijux GNSS changelog](../../CHANGELOG.md).
This package adheres to [Semantic Versioning](https://semver.org) and the
[Keep a Changelog](https://keepachangelog.com/en/1.0.0/) format.

---

## 0.1.0 - 2026-07-19

### Added

- Implemented broadcast navigation decoding for GPS LNAV and CNAV, Galileo
  I/NAV and F/NAV, GLONASS strings, and BeiDou D1 and D2 with packet, page,
  parity, timing, and assembly validation.
- Added RINEX observation and navigation readers and writers, SP3 and CLK
  precise-product readers, ANTEX antenna models, and Bias-SINEX records with
  source metadata and uncertainty preservation.
- Added GPS, Galileo, GLONASS, and BeiDou broadcast orbit and clock propagation
  with analytical state derivatives, convergence evidence, ephemeris selection,
  uncertainty, and explicit rejection reasons.
- Defined GPST, GST, BDT, GLONASS, UTC, and TAI conversion behavior with leap
  seconds, epoch context, rollover resolution, and refusal of ambiguous time.
- Implemented ionosphere, troposphere, group-delay, phase-windup, antenna,
  tide, bias, dual-frequency, geometry-free, ionosphere-free, Melbourne-Wubbena,
  and narrow-lane correction models.
- Added weighted least-squares positioning, dilution-of-precision diagnostics,
  robust weighting, trajectory and solution smoothing, EKF state estimation,
  covariance propagation, and solver-conditioning evidence.
- Implemented RAIM integrity monitoring with protection levels, residual tests,
  satellite exclusion, multi-fault separation hypotheses, and fail-closed
  unresolved-fault behavior.
- Implemented RTK single and double differences, reference switching,
  covariance transformation, float baselines, integer decorrelation and search,
  partial fixes, ambiguity hold, and GLONASS FDMA bias evidence.
- Implemented PPP state estimation with precise-product policy, stochastic
  controls, measurement weighting, ambiguity evidence, ionosphere scaling,
  lifecycle resets, convergence, and discontinuity reporting.
- Published a curated `api` module and `NavEngine` contract that expose
  products, models, estimators, uncertainty, integrity, solution claims, and
  typed refusal evidence behind an optional precise-products feature.
