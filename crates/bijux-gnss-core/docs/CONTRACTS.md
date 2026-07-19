# Contracts

`bijux-gnss-core` owns the stable records that higher-level crates exchange. Those records are not
incidental data bags; they are the shared workspace contract layer.

The module-to-contract ownership map is recorded in the [contract map](CONTRACT_MAP.md).

## Artifact contracts

The artifact layer owns:
- `ArtifactHeaderV1`
- `ArtifactV1`
- `ArtifactKind`
- acquisition, observation, tracking, navigation, and support-matrix payload versions
- payload validation via `ArtifactPayloadValidate` and `ArtifactValidate`

This contract family defines what gets persisted, inspected, exported, and validated across the
workspace.

## Observation contracts

The observation layer owns:
- sample frames and receiver roles
- acquisition requests/results and explainability payloads
- tracking epochs and transitions
- observation epochs, decisions, manifests, and uncertainty classes
- navigation-observation metadata and measurement rejection reasons
- single- and double-difference records

These contracts must stay solver-neutral enough for both `receiver` and `nav` to share them.

## Navigation-solution contracts

The navigation family owns:
- `NavSolutionEpoch`
- `NavResidual`
- `NavConstellationResidualRms`
- `InterSystemBias`
- solution-validity, lifecycle-state, and refusal-class records

These contracts describe the result of navigation processing, not how the solver internally arrived
there.

## Foundational contracts

The foundational layer owns:
- identity records for constellations, satellites, signals, and frequency channels
- time-system records and conversion helpers
- strong physical units and unit conversions
- WGS-84 coordinate types and transforms
- diagnostics and canonical error taxonomies

These contracts are intentionally low-level and reusable.

Their serialized representation must remain portable across crates and persisted artifacts. The
crate-wide serialization rules are recorded in the [serialization guide](SERIALIZATION.md).

## Diagnostics and support contracts

The crate also owns:
- shared diagnostic event and summary records
- stable diagnostic code metadata
- support-matrix status and row contracts exchanged across runtime, infrastructure, and CLI layers

These surfaces are detailed in the [diagnostic guide](DIAGNOSTICS.md) and
[support matrix](SUPPORT_MATRIX.md).

## Configuration contracts

The configuration layer owns:
- schema versioning
- configuration validation traits
- validation report shape

It defines what “valid configuration” means at the contract level, not how the CLI finds or writes
configuration files.

## Non-goals

This crate does not own:
- raw sample transport
- filesystem manifests or run directories
- DSP implementations
- orbit, atmosphere, PPP, or RTK estimation behavior
- receiver scheduling or CLI workflows
