# Commands

This file is the single source of truth for the command families owned by the `bijux` binary.

## Top-level command family

- `bijux gnss ...`

## Stable workflow families

- acquisition and capture inspection workflows
- run-pipeline workflows
- artifact validation, explanation, and conversion workflows
- synthetic signal generation and export workflows
- navigation-format and RINEX workflows
- configuration validation and schema workflows
- diagnostics and reporting workflows
- validation workflows over capture, synthetic IQ, synthetic navigation, and bias/reference data
- analysis and comparison workflows over produced runs and artifacts

## Command-shape ownership

The CLI crate owns:
- command names
- argument and flag structure
- report-format selection
- operator-facing output shape

The underlying science and persistence behavior still belong to the lower-level crates the commands
delegate into.

The workflow-composition boundary for these command families is described in
[WORKFLOWS.md](WORKFLOWS.md).
