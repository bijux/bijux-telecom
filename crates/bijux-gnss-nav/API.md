# bijux-gnss-nav API

Stable public API surface exposed via `crates/bijux-gnss-nav/src/api.rs`.

Navigation models
- `GpsEphemerisV1`, `GpsEphemeris`, `NavClockModel`.
- Measurement models: `PseudorangeMeasurement` and related helpers.

State + math
- `Matrix`, `StateCovariance`, `ProcessNoiseConfig`.

Bias providers
- `CodeBiasProvider`, `PhaseBiasProvider`.

Writers
- `write_rinex_obs`, `write_rinex_nav`.
