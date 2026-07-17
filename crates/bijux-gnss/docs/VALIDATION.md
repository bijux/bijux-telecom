# Validation

`bijux-gnss` owns the operator-facing validation workflows exposed by the `bijux` binary.

## Validation responsibilities

The CLI currently owns workflow entrypoints for validation over:

- capture and raw-IQ inputs
- configuration files
- synthetic IQ and synthetic navigation outputs
- bias and reference-data inputs

## Boundary rule

The CLI owns how validation workflows are invoked and presented to operators. It does not own the
underlying validation science or persisted artifact contracts.
