---
title: Module Map
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# Module Map

This page maps the main scientific families in `bijux-gnss-nav` to the code
that owns them.

## Top-Level Surface

- `src/api.rs` is the curated downstream surface and the first place to check
  whether a type or helper is meant to be consumed outside the crate
- `src/lib.rs` states the durable crate map: `formats`, `orbits`,
  `corrections`, `estimation`, `linalg`, `models`, and `time`

## Product Interpretation

- `src/formats/` owns navigation-message and reference-product parsing
- `src/formats/gps_navigation/` owns LNAV and CNAV families
- `src/formats/galileo_navigation/` owns FNAV and INAV families
- `src/formats/beidou_navigation/` owns B1I and D2 families
- `src/formats/glonass_navigation/` owns GLONASS navigation decoding
- `src/formats/rinex_navigation/` and `src/formats/rinex_observation/` own
  RINEX-domain interpretation
- `src/formats/precise_products/` owns SP3, CLK, ANTEX, and bias SINEX parsing

## Orbit And Time Interpretation

- `src/orbits/` owns constellation-specific broadcast orbit interpretation,
  precise-state helpers, and ephemeris provider seams
- `src/time.rs` plus `src/time/rollover.rs` own GNSS-specific time-scale and
  rollover interpretation above `core`

## Correction Law

- `src/corrections/atmosphere.rs` owns atmosphere configuration and ZTD support
- `src/corrections/biases.rs` owns code and phase bias provider contracts
- `src/corrections/broadcast_group_delay.rs` and
  `src/corrections/broadcast_ionosphere_residuals.rs` own broadcast-driven
  correction families
- `src/corrections/geometry_free.rs`, `iono_free_code.rs`,
  `iono_free_phase.rs`, `measured_ionosphere.rs`, `melbourne_wubbena.rs`, and
  `narrow_lane.rs` own reusable diagnostic and combination families

## Estimation Families

- `src/estimation/ekf/` owns reusable filter primitives and measurement-model
  seams
- `src/estimation/position/` owns positioning, integrity, RAIM, navigation
  runtime-neutral filtering, trajectory reconstruction, and solution reporting
- `src/estimation/ppp/` owns PPP configuration, state, filter evolution,
  quality evidence, and product policy
- `src/estimation/rtk/` owns differencing, ambiguity logic, baseline solving,
  antenna correction, and RTK quality evidence
- `src/estimation/solution_claims.rs` owns advanced claim and downgrade
  reporting over PPP and RTK outputs

## Supporting Models

- `src/models/antenna.rs`, `atmosphere.rs`, `celestial.rs`,
  `ocean_tide_loading.rs`, and `solid_earth_tide.rs` own supporting physical
  models
- `src/models/nequick/` owns NeQuick-specific data and model execution

## Shared Small Math

- `src/linalg.rs` owns crate-local matrix support used by estimators where a
  compact internal implementation is preferable to pushing the full crate
  boundary downward
