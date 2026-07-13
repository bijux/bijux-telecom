# bijux-gnss-nav API

Stable public API surface exposed via `crates/bijux-gnss-nav/src/api.rs`.

Navigation models
- `GpsEphemerisV1`, `GpsEphemeris`, `NavClockModel`.
- `GpsEphemeris.week` is the resolved full GPS week.
- LNAV ephemeris decoders require explicit reference-week context to expand the 10-bit broadcast week.
- Measurement models: `PseudorangeMeasurement` and related helpers.
- `NavigationFilter`, `NavigationFilterConfig`, `NavigationFilterThresholds`.
- `PositionSatelliteState`, `position_satellite_state_from_observation`.

State + math
- `Matrix`, `StateCovariance`, `ProcessNoiseConfig`.
- RTK execution helpers: `build_sd`, `build_dd`, `choose_ref_sat`, `solve_baseline_dd`.
- RTK quality helpers: `baseline_from_ecef`, `dd_residual_metrics`,
  `evaluate_rtk_fixed_baseline_guard`.

Bias providers
- `CodeBiasProvider`, `PhaseBiasProvider`.

Writers
- `write_rinex_obs`, `write_rinex_nav`.
