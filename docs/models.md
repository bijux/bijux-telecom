# GNSS Model Manifest

## Signal Model Assumptions
- GPS L1 C/A code only.
- GPS L1 C/A PRNs 1-32 are validated against the published GPS SPS Table 2-1 tap pairs, delays, and first-ten-chip references.
- GPS L1 C/A period length is fixed at 1023 chips and the generated sequence repeats exactly on that boundary for PRNs 1-32.
- GPS L1 C/A periodic autocorrelation for PRNs 1-32 is validated with a 1023-chip main peak and non-zero sidelobes limited to `-65`, `-1`, and `63`, with maximum absolute sidelobe magnitude `65`.
- GPS L1 C/A periodic cross-correlation for every distinct PRN pair in 1-32 is validated with values limited to `-65`, `-1`, and `63`, with maximum absolute magnitude `65`, so no false full-period main peak appears between different codes.
- GPS L1 C/A sampled-code generation advances chip phase from exact `code_rate_hz / sample_rate_hz` progression, so arbitrary sample rates and chunk boundaries preserve the same wrapped chip sequence without rounded samples-per-chip drift.
- The sampled-code phase model is validated at the 60-second boundary: direct elapsed-time advance, chunked sample-count advance, and synthetic receiver epoch alignment agree within a documented floating-point tolerance, and the resulting sampled C/A block remains unchanged.
- Carrier wipeoff uses an absolute-time-aware NCO keyed to sample index, so long-offset acquisition and tracking frames preserve bounded prompt-phase error instead of resetting carrier phase at each local block origin.
- Synthetic carrier generation and acquisition both interpret Doppler relative to the configured IF, using the same `carrier_hz = intermediate_freq_hz + doppler_hz` mapping for zero-IF and high-IF captures.
- Synthetic GPS L1 C/A navigation data is modeled as a deterministic 50 bps sign modulation: when enabled, the data-bit sign alternates every 20 ms starting positive at sample zero, and the truth bundle records the exact sample-aligned bit segments.
- Ideal spreading codes (±1) without front-end distortions.
- Additive noise approximated in tracking metrics.

## Tracking Loop Assumptions
- Scalar tracking by default; vector tracking may be enabled via config.
- DLL/PLL/FLL loops assume near-constant dynamics over 1 ms integration.
- Cycle slips detected via phase residual thresholds.
- Prompt-phase continuity treats deterministic 20 ms nav-bit sign inversions as data transitions rather than carrier cycle slips, so 1 ms coherent tracking can span GPS LNAV bit boundaries without false slip reports.

## Navigation Measurement Model
- Pseudorange observations derived from code phase and clock bias.
- Weighted least squares / EKF use elevation/CN0-based weighting.
- Broadcast ephemerides used when precise products are unavailable.

## Limitations
- No ionosphere/troposphere correction by default.
- Limited constellation support (GPS-focused pipelines).
- Navigation accuracy degrades with poor geometry or missing ephemerides.
