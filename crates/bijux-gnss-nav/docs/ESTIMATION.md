# Estimation

`bijux-gnss-nav` owns navigation estimation as a scientific boundary, not as a receiver runtime.

## Estimation families

`src/estimation/` owns several durable families:

- `ekf/` for reusable state-estimation primitives
- `position/` for positioning, integrity, smoothing, weighting, and runtime-neutral solution logic
- `ppp/` for precise point positioning filters, measurements, and lifecycle evidence
- `rtk/` for ambiguity, baseline, differencing, fix policy, and quality surfaces
- `solution_claims.rs` for advanced claim, downgrade, and support-matrix reporting

## Boundary rules

- Estimation here may consume observations, satellite states, and corrections.
- Stage scheduling, sample-flow orchestration, and persisted run layout belong elsewhere.
- Solver outputs may be consumed by `receiver`, `infra`, and CLI crates, but the estimation logic
  itself stays owned here.

## Public-surface discipline

The API exposes broad estimation capabilities because higher-level crates genuinely need them, but
solver-local helpers should stay private. If an item is only meaningful to one internal solver
path, it should not become part of the shared navigation API.
