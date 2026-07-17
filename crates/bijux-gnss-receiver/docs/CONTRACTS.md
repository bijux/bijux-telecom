# Contracts

`bijux-gnss-receiver` owns runtime receiver contracts.

## Configuration contract

The crate owns:
- `ReceiverConfig`
- `ReceiverPipelineConfig`
- runtime options and runtime-side sink contracts

These define how a receiver run is configured and what runtime hooks are available.

## Stage execution contract

The pipeline layer owns:
- acquisition, tracking, observation, and navigation stage entrypoints
- typed handoff between those stages
- stage report and stage statistics helpers

This contract is about runtime composition, not repository persistence.

## Artifact contract

`RunArtifacts` and the observation/tracking/navigation report types define the in-memory result of a
receiver run before infrastructure crates persist or inspect it.

## Validation and simulation contract

When `nav` is enabled, the crate also owns receiver-boundary validation helpers, synthetic scenario
execution, and covariance realism support. These are runtime-facing validation capabilities, not the
repository-facing validation contract owned by `infra`.
