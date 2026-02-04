# Determinism Policy

## Modes
- Deterministic mode enforces stable ordering, fixed seeds, and deterministic FFT planning where possible.

## Expectations
- JSONL artifacts should be identical in deterministic mode on the same platform.
- Cross-platform results must be within numeric tolerances (see NUMERICS.md).

## Current Constraints
- Floating-point differences can occur across CPU/OS.
- Use deterministic mode for regression baselines.

