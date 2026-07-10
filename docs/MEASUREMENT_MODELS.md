# Measurement Models and Pipeline Contract

This document defines the measurement equations, units, sign conventions, and the formal
measurement pipeline contract used in bijux-gnss. The goal is to make all measurement handling
explicit and testable across acquisition, tracking, observation building, and navigation.

## Pipeline Contract (Formal)
Every measurement emitted by the pipeline MUST satisfy:
- Time reference: receiver time `t_rx` (seconds, GPS time), tagged per epoch.
- Sign conventions:
  - Pseudorange is positive, increasing with geometric range.
  - Doppler is positive when carrier phase is increasing.
  - Carrier phase is in cycles; continuity between epochs is expected except at explicit
    ambiguity-arc boundaries.
- Units:
  - Pseudorange: meters.
  - Carrier phase: cycles.
  - Doppler: Hz.
  - CN0: dB-Hz.
- Each measurement must declare:
  - Signal identity (`SigId`: constellation + band + code).
  - Quality flags (lock flags, cycle slip, and explicit reject reason when applicable).
  - Variance or error model where available.

The contract is enforced in validation and navigation (reject reasons, chi-square gating, and
sanity checks). Any violation must emit diagnostics and mark the measurement as rejected.

## Code (Pseudorange)
- Model: `P = ρ + c(δt_r - δt_s) + T - I + ε`
- Units: meters
- Notes: `ρ` is geometric range, `T` troposphere delay, `I` ionosphere delay (positive for code).
- Receiver contract: absolute code pseudorange is emitted when tracking carries a resolved
  `SignalDelayAlignment` whole-code-period count. If alignment is absent, the observation remains a
  receiver-epoch fallback estimate and must not emit `ObsSignalTiming`.

## Carrier Phase
- Model: `φ = (ρ + c(δt_r - δt_s) + T - I) / λ + N + ε`
- Units: cycles
- Notes: `N` is integer ambiguity. Carrier phase is continuous and must remain consistent with
  Doppler integration.
- Receiver contract: observation metadata declares `carrier_phase_model =
  "tracked_carrier_cycles"` and one of these continuity labels:
  `arc_start`, `continuous`, `reset_after_cycle_slip`, `reset_after_unlock`,
  `reset_after_discontinuity`, or `unusable`.
- Arc contract: `carrier_phase_arc_start_epoch_idx` and
  `carrier_phase_arc_start_sample_index` identify the start of the current ambiguity arc. They
  change only when a new usable carrier-phase arc begins.

## Doppler
- Model: `f_D = d/dt(φ)`
- Units: Hz
- Sign convention: positive Doppler implies increasing carrier phase.
- Receiver contract: acquisition and tracking may carry an absolute in-band `carrier_hz`, while
  reported Doppler observations are the IF-relative offset `carrier_hz - intermediate_freq_hz`.
- Observation metadata contract: observation metadata declares `doppler_model =
  "tracked_carrier_hz_minus_intermediate_freq"` for Doppler emitted from tracked carrier
  frequency and configured intermediate frequency.
- Acquisition search contract: each carrier bin is formed explicitly as
  `carrier_hz = intermediate_freq_hz + doppler_hz`, so zero-IF and high-IF inputs use the same
  Doppler-bin semantics and only differ by the configured carrier offset.

## Receiver Clock Model
- State: receiver clock bias `δt_r` and drift `δṫ_r` (seconds and seconds/second).
- Dynamics (discrete): `δt_r(k+1) = δt_r(k) + δṫ_r(k) * Δt + w_b`,
  `δṫ_r(k+1) = δṫ_r(k) + w_d`.
- Bias is estimated in WLS/PPP; drift is tracked explicitly in filtering models.

## Satellite Clock Model
- Broadcast GPS clock correction is carried as `GpsSatelliteClockCorrection`.
- Applied broadcast bias:
  `δt_s = base_bias_s + relativistic_s - group_delay_s`.
- Broadcast base bias is the LNAV polynomial:
  `base_bias_s = af0 + af1 * Δt + af2 * Δt²`, with `Δt = t_tx - toc`.
- Broadcast drift and drift rate are exposed explicitly as:
  `drift_s_per_s = af1 + 2 * af2 * Δt` and `drift_rate_s_per_s2 = 2 * af2`.
- `GpsSatState` carries the full `clock_correction` alongside ECEF position so pseudorange
  generation and navigation solvers use one consistent clock surface.
- When precise CLK products are available, `ProductsProvider::clock_correction` supplies a
  bias-only correction with zero relativistic and group-delay terms; callers must not reapply
  broadcast-only terms on top of a precise clock bias.

## Noise/Weighting
- CN₀- and elevation-weighted variance is used in navigation solvers.
- See `NavigationWeightingConfig` for the current policy knobs.
