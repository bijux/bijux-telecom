# Contracts

`bijux-gnss-infra` owns infrastructure contracts around repository state and persisted artifacts.

## Dataset contract

The dataset layer owns:
- dataset registry records
- raw-IQ metadata resolution
- coordinate parsing and recorded-capture provenance helpers

## Run-layout contract

The run-layout layer owns:
- run directory identity and layout
- manifest and run-report shapes
- artifact headers for persisted outputs
- run history entry shape and append semantics

This is the repository’s durable execution footprint contract.

## Artifact inspection contract

The artifact-inspection layer owns:
- artifact kind detection
- artifact validation entrypoints
- artifact explanation summaries

It describes how persisted artifacts are interrogated after they are produced.

## Override and sweep contract

The override layer owns typed profile mutation, while the experiment/sweep layer owns stable
parameter expansion over experiment specs. These contracts let callers vary configurations without
building ad hoc string manipulation.

## Validation adapter contract

`validate_reference` and related API re-exports own the infrastructure-facing bridge between
persisted artifacts and validation/reference comparison flows.
