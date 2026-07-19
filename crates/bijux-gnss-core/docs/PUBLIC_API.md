# Public API

`bijux-gnss-core` publishes one deliberate downstream surface: `bijux_gnss_core::api`.

## Public module policy

- `src/lib.rs` exposes `pub mod api;`
- implementation modules remain private
- new public structs and free functions must be re-exported through `api.rs`

That policy is enforced by `tests/public_api_guardrail.rs`.

## Public API families

### Artifact contracts

`api.rs` re-exports:
- versioned acquisition, observation, tracking, navigation, and support-matrix artifact payloads
- artifact headers, kind enums, read policy, payload-validation traits, and conversion helpers

Use this family when a downstream crate needs stable serialized envelopes or payload validation.

### Configuration and diagnostics

`api.rs` exposes:
- `BijuxGnssConfig`, `SchemaVersion`, `ValidateConfig`, and `ValidationReport`
- diagnostic codes, events, summaries, severity levels, and aggregation helpers
- canonical error enums such as `ConfigError`, `NavError`, `TrackError`, and `SignalError`

Use this family when a crate needs to validate configs or emit structured failure information with
shared workspace semantics.

### Foundational physical types

`api.rs` exposes:
- `Constellation`, `SatId`, `SigId`, signal-band/code/spec records, and carrier-frequency constants
- `Epoch`, `GpsTime`, `UtcTime`, `TaiTime`, `ReceiverSampleTrace`, and leap-second records
- `Meters`, `Seconds`, `Cycles`, `Hertz`, `Chips`, and conversion helpers
- `Ecef`, `Enu`, `Llh`, and geodetic conversion helpers

These types are the workspace’s shared language for GNSS state.

### Observation and navigation records

`api.rs` exposes:
- acquisition request/result and explainability records
- observation-epoch and sample-frame contracts
- differencing records and observation-quality metadata
- navigation-solution, residual, inter-system-bias, and lifecycle-state records
- support-matrix and tracking state contracts

Use these records to exchange pipeline state across crates without coupling to implementation layout.

## Extension rules

1. Add a type to `api.rs` only when it is genuinely cross-crate.
2. Keep implementation helpers private if the type is local to one module.
3. Prefer extending an existing API family over creating a new public namespace.
4. If a new public export changes serialized meaning or downstream expectations, update
   [contract guide](CONTRACTS.md) and [invariant guide](INVARIANTS.md) in the same change set.
5. If a new public export changes the contract-family boundary, update the [contract map](CONTRACT_MAP.md)
   so module ownership stays explicit.
