# Numerical Budgets

This document defines numeric tolerance budgets for deterministic verification and regression.

## Acquisition

- Doppler bin accuracy: within 1 bin (`<= doppler_step_hz`)
- Code phase accuracy: within 1 sample or configured decimation
- Peak ratio tolerance: `>= configured threshold`

## Tracking

- PLL jitter: `<= 0.5 rad` (default)
- DLL jitter: `<= 0.2 chips` (default)
- Cycle slip detection: no false positives above thresholds

## Navigation (PVT)

- Residual RMS: within configured gating (`navigation.ppp.residual_gate_m`)
- Convergence: stable within `convergence_sigma_h_m` and `convergence_sigma_v_m`

## RTK / PPP

- Fix ratio: meets configured thresholds
- Baseline RMS: within expected synthetic truth tolerance

## Determinism

When `--deterministic` is enabled, repeated runs should produce identical artifact hashes.
If floating-point noise is present, comparisons must be within the tolerances above.
