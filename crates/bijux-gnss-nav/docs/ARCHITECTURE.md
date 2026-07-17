# Architecture

`bijux-gnss-nav` is the workspace navigation-science crate.

## Source map

- `src/api.rs` is the curated downstream surface.
- `src/orbits/` owns broadcast and precise orbit state, ephemeris records, and satellite
  uncertainty helpers.
- `src/formats/` owns:
  - GPS LNAV and CNAV decoding
  - Galileo FNAV and INAV decoding
  - BeiDou and GLONASS navigation decoding
  - RINEX navigation and observation parsing
  - precise-product formats such as SP3, CLK, ANTEX, and bias SINEX
- `src/corrections/` owns atmosphere, broadcast ionosphere, group-delay, combination, windup, and
  dual-frequency correction helpers.
- `src/estimation/` owns:
  - EKF primitives
  - position and integrity solvers
  - PPP measurement and filter logic
  - RTK ambiguity, baseline, and execution logic
  - solution-claim reporting and uncertainty helpers
- `src/models/` owns supporting physical models such as antenna, atmosphere, tides, and NeQuick.
- `src/linalg.rs` owns small matrix support used by estimators.
- `src/time/` owns navigation-time helpers and rollover resolution logic.

## Dependency direction

This crate depends on `bijux-gnss-core` and `bijux-gnss-signal`. Higher-level crates such as
`receiver`, `infra`, `testkit`, and the CLI build on top of it.

## Test map

The test suite is broad because the crate spans several scientific surfaces:
- format and decoder tests
- orbit and precise-product reference tests
- correction and dual-frequency tests
- SPP, PPP, RAIM, and RTK solution tests
- public-data and reference-coordinate tests
- guardrail, fault-injection, and long-run stability tests

## Design constraints

- parsing logic should stay near the format families that own it
- estimation logic should stay separated from repository and runtime concerns
- navigation-domain helpers belong here only when they are reused or scientifically central, not
  simply because they were awkward to place elsewhere
