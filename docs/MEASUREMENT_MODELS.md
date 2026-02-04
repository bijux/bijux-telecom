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
  - Carrier phase is in cycles; continuity between epochs is expected except on cycle slip.
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

## Carrier Phase
- Model: `φ = (ρ + c(δt_r - δt_s) + T - I) / λ + N + ε`
- Units: cycles
- Notes: `N` is integer ambiguity. Carrier phase is continuous and must remain consistent with Doppler integration.

## Doppler
- Model: `f_D = d/dt(φ)`
- Units: Hz
- Sign convention: positive Doppler implies increasing carrier phase.

## Receiver Clock Model
- State: receiver clock bias `δt_r` and drift `δṫ_r` (seconds and seconds/second).
- Dynamics (discrete): `δt_r(k+1) = δt_r(k) + δṫ_r(k) * Δt + w_b`,
  `δṫ_r(k+1) = δṫ_r(k) + w_d`.
- Bias is estimated in WLS/PPP; drift is tracked explicitly in filtering models.

## Noise/Weighting
- CN₀- and elevation-weighted variance is used in navigation solvers.
- See `NavigationWeightingConfig` for the current policy knobs.
