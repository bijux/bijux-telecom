# Measurement Models

This document defines the measurement equations, units, and sign conventions used in bijux-gnss.

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

## Noise/Weighting
- CN₀- and elevation-weighted variance is used in navigation solvers.
- See `NavigationWeightingConfig` for the current policy knobs.

