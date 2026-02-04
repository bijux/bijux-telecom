# GNSS Model Manifest

## Signal Model Assumptions
- GPS L1 C/A code only.
- Ideal spreading codes (±1) without front-end distortions.
- Additive noise approximated in tracking metrics.

## Tracking Loop Assumptions
- Scalar tracking by default; vector tracking may be enabled via config.
- DLL/PLL/FLL loops assume near-constant dynamics over 1 ms integration.
- Cycle slips detected via phase residual thresholds.

## Navigation Measurement Model
- Pseudorange observations derived from code phase and clock bias.
- Weighted least squares / EKF use elevation/CN0-based weighting.
- Broadcast ephemerides used when precise products are unavailable.

## Limitations
- No ionosphere/troposphere correction by default.
- Limited constellation support (GPS-focused pipelines).
- Navigation accuracy degrades with poor geometry or missing ephemerides.
