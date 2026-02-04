# bijux-gnss-core API

Stable public API surface exposed via `crates/bijux-gnss-core/src/api.rs`.

Configuration + validation
- `SchemaVersion`: schema version wrapper.
- `ValidateConfig`, `ValidationReport`: config validation traits and results.
- `BijuxGnssConfig`: combined receiver+navigation config wrapper.

Artifacts
- `ArtifactHeaderV1`: artifact header schema.
- `ArtifactType`: artifact type identifiers.
- `ArtifactReadPolicy`: reader policy.

Signals + IDs
- `Constellation`, `SignalBand`, `SatId`.

Measurements + epochs
- `AcqResultV1`, `TrackEpochV1`, `ObsEpochV1`, `NavSolutionEpochV1` and related record structs.

Diagnostics + errors
- `DiagnosticEvent`, `DiagnosticSeverity`.
- `InputError`, `ConfigError`, `SignalError`, `AcqError`, `TrackError`, `NavError`.
