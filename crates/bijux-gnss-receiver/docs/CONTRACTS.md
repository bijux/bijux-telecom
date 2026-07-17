# Contracts

`bijux-gnss-receiver` owns runtime receiver contracts.

## Configuration contract

The crate owns:
- `ReceiverConfig`
- `ReceiverPipelineConfig`
- runtime options and runtime-side sink contracts

These define how a receiver run is configured and what runtime hooks are available.

The runtime-composition layer behind those contracts is detailed in [RUNTIME.md](RUNTIME.md).

## Stage execution contract

The pipeline layer owns:
- acquisition, tracking, observation, and navigation stage entrypoints
- typed handoff between those stages
- stage report and stage statistics helpers

This contract is about runtime composition, not repository persistence.

The runtime stage boundary and handoff responsibilities are detailed in [PIPELINE.md](PIPELINE.md).

## Artifact contract

`RunArtifacts` and the observation/tracking/navigation report types define the in-memory result of a
receiver run before infrastructure crates persist or inspect it.

The in-memory artifact boundary is detailed in [ARTIFACTS.md](ARTIFACTS.md).

## Validation and simulation contract

When `nav` is enabled, the crate also owns receiver-boundary validation helpers, synthetic scenario
execution, and covariance realism support. These are runtime-facing validation capabilities, not the
repository-facing validation contract owned by `infra`.

The execution-time source, sink, and clock seams that support this boundary are detailed in
[PORTS.md](PORTS.md).
