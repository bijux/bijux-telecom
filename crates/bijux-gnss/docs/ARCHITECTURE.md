# Architecture

`bijux-gnss` is the operator command boundary for the GNSS workspace.

## Source map

- `src/main.rs` assembles the CLI by including the command, runtime, catalog, and reporting modules.
- `src/lib.rs` is a small façade over curated lower-level crates.
- `src/cli/command_catalog/` owns stable command and argument shapes.
- `src/cli/commands/` owns command execution handlers such as ingest, synthetic generation,
  artifact workflows, run-pipeline execution, analysis, and validation.
- `src/cli/command_runtime/` owns runtime-environment setup, dataset inspection, and report support
  needed during command execution.
- `src/cli/command_support/` owns helper adapters such as artifact loading, capture windows,
  navigation outputs, receiver artifacts, and raw-IQ quality support.
- `src/cli/report.rs` owns operator-facing output rendering.

## Dependency direction

This crate sits at the top of the Rust package stack:
- it may depend on `core`, `receiver`, `signal`, `infra`, and optional `nav`
- lower-level crates must not depend on this crate

## Test map

- CLI integration tests cover acquisition configuration, raw-IQ metadata, synthetic IQ export and
  validation, navigation decode, RINEX workflows, and capture/config validation.
- `tests/integration_guardrails.rs` keeps the crate aligned with workspace guardrails.

## Design constraints

- command handlers should orchestrate stable lower-level APIs rather than reimplementing them
- command modules should be grouped by operator workflow, not by delivery history
- CLI reporting and output shape are owned here; product computation is not
