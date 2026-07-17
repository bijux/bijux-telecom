# Workflows

`bijux-gnss` owns operator-facing workflow composition across the lower-level GNSS crates.

## Workflow families

The CLI currently owns durable workflow families such as:

- acquisition and capture inspection
- run-pipeline execution
- artifact validation, explanation, and conversion
- synthetic signal generation and export
- navigation decode and RINEX-oriented flows
- configuration validation and diagnostics
- analysis and comparison over produced runs and artifacts

## What the CLI owns

For each workflow, this crate owns:

- command naming and flag shape
- top-level argument interpretation
- runtime setup and report formatting
- the user-facing sequencing of lower-level crate calls

## What the CLI does not own

- signal and DSP behavior
- receiver stage math
- navigation solver semantics
- persisted run-layout and manifest contracts

Those surfaces stay in their owning crates even when the operator experiences them through this
binary.
