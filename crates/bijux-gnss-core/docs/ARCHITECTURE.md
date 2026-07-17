# Architecture

`bijux-gnss-core` is the shared contract foundation for the GNSS workspace. It does not run a
pipeline. It defines the language the rest of the workspace uses when they talk about samples,
signals, observations, diagnostics, navigation solutions, artifact payloads, time systems, and
physical units.

## Source map

- `src/api.rs` is the only curated downstream entrypoint.
- `src/artifact/` owns versioned artifact envelopes and payload validation rules.
- `src/config.rs` owns configuration schema versions and validation reporting primitives.
- `src/conventions.rs` owns scientific sanity helpers used to check observation and solution
  consistency.
- `src/diagnostic/` owns diagnostic codes, severity levels, and aggregation helpers.
- `src/error.rs` owns canonical error categories shared across downstream crates.
- `src/geo.rs` owns WGS-84 coordinate transforms and geometry helpers.
- `src/ids.rs` owns constellation, satellite, signal, and registry identity types.
- `src/nav_solution.rs` owns navigation-solution records, residuals, and inter-system bias payloads.
- `src/observation/` owns acquisition, tracking, navigation-observation, and differencing records.
- `src/observation_quality.rs` owns lock, covariance, and cycle-slip quality metadata.
- `src/stats.rs` owns simple statistical summaries shared across higher layers.
- `src/support_matrix.rs` owns supported-signal inventory contracts.
- `src/time.rs` owns GPS, UTC, TAI, receiver-sample time, and leap-second contracts.
- `src/units.rs` owns strong physical units and conversion helpers.

## Dependency direction

This crate sits near the bottom of the workspace:
- higher-level crates may depend on `bijux-gnss-core`
- `bijux-gnss-core` must not depend on `signal`, `nav`, `receiver`, `infra`, CLI, or testkit crates

That rule keeps the foundational types usable everywhere without importing pipeline behavior.

## Public surface discipline

`src/lib.rs` keeps implementation modules private and exposes one public module, `api`. The public
API guardrail test enforces that newly public structs and free functions are deliberately re-exported
through `api.rs` instead of leaking by accident from implementation modules.

## Test map

- `tests/public_api_guardrail.rs` locks curated API exposure.
- `tests/nav_artifact_validation.rs` validates navigation artifact payload rules.
- `tests/tracking_artifact_validation.rs` validates tracking artifact payload rules.
- `tests/prop_timekeeping.rs` and `tests/prop_timekeeping.proptest-regressions` lock timekeeping
  behavior with property tests.
- `tests/integration_guardrails.rs` carries workspace guardrail coverage for this crate.
- `tests/data/obs_fixture.jsonl` is a stable artifact validation fixture used by downstream checks.

## Design constraints

- Foundational contracts must remain serializable and stable enough for downstream crates to store,
  compare, and validate them.
- Scientific helpers here should stay pure and reusable. Runtime state, filesystem state, and
  command policy belong elsewhere.
- If a type or helper is only useful to one higher-level crate, it probably does not belong here.
