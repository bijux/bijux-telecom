# Correctness Budgets

These budgets define the nominal numerical tolerances enforced by `bijux gnss validate`.
They are intended to be conservative and should be tightened as the pipeline matures.

## Acquisition
- Doppler error: ≤ 500 Hz
- Code phase error: ≤ 5 samples

## Tracking
- Carrier frequency jitter (stddev): ≤ 50 Hz

## Ephemeris Decode
- LNAV parity pass rate: ≥ 0.90

## PVT
- Max iterations per epoch: ≤ 10
- Residual RMS: ≤ 50 m
- Rejected measurement ratio: ≤ 0.5
- NaN count: 0 allowed
- Minimum lock epochs before nav update: ≥ 3

If a budget is violated, the validation report includes a `budget_violations` entry.

## Time Consistency Checks

The validation report also includes a `time_consistency` section that summarizes:
- Epoch index monotonicity and gaps.
- Sample index monotonicity.
- Sample cadence vs the configured sample rate (per-epoch sample step).

If cadence mismatches are detected, the report includes a warning and the observed mean
sample step to help diagnose rate drift or framing errors.
