# Invariants

This file records the behaviors downstream crates should be able to rely on when they consume
`bijux-gnss-core`.

## Public-surface invariants

- Public structs and free functions must be deliberately re-exported through `src/api.rs`.
- Higher-level crates should not depend on implementation module paths directly.

Enforced by `tests/public_api_guardrail.rs`.

## Artifact invariants

- Navigation artifact payload validation rejects inconsistent model versions and satellite counts.
- Navigation artifact payload validation catches non-finite DOPs, non-finite covariances, and
  inconsistent clock-bias units.
- Tracking and navigation artifact validators remain responsible for payload coherence, not just
  type shape.

Enforced by `tests/nav_artifact_validation.rs` and `tests/tracking_artifact_validation.rs`.

## Time invariants

- GPS, UTC, and receiver-sample time conversions stay property-tested and regression-locked.
- Timekeeping helpers must preserve deterministic behavior under the existing proptest corpus.

Enforced by `tests/prop_timekeeping.rs` and
`tests/prop_timekeeping.proptest-regressions`.

## Boundary invariants

- `bijux-gnss-core` remains free of higher-level workspace crate dependencies.
- The crate stays a contract foundation rather than becoming a runtime or orchestration surface.

Guardrail coverage is exercised in `tests/integration_guardrails.rs`.
