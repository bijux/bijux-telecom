# GNSS Model Manifest

## Signal Model Assumptions
- GPS L1 C/A code only.
- GPS L1 C/A PRNs 1-32 are validated against the published GPS SPS Table 2-1 tap pairs, delays, and first-ten-chip references.
- GPS L1 C/A period length is fixed at 1023 chips and the generated sequence repeats exactly on that boundary for PRNs 1-32.
- GPS L1 C/A periodic autocorrelation for PRNs 1-32 is validated with a 1023-chip main peak and non-zero sidelobes limited to `-65`, `-1`, and `63`, with maximum absolute sidelobe magnitude `65`.
- GPS L1 C/A periodic cross-correlation for every distinct PRN pair in 1-32 is validated with values limited to `-65`, `-1`, and `63`, with maximum absolute magnitude `65`, so no false full-period main peak appears between different codes.
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
