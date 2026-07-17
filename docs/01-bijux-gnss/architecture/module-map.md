---
title: Module Map
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# Module Map

This page maps the main command-boundary families in `bijux-gnss` to the code
that owns them.

## Top-Level Surface

- `src/main.rs` assembles the binary surface by including and wiring the CLI
  modules
- `src/lib.rs` owns the thin package facade over lower-level crates

## Command Shape

- `src/cli/command_catalog/` owns stable command families and argument shapes
- `src/cli/command_catalog/artifact_commands.rs`,
  `configuration_commands.rs`, `diagnostics_commands.rs`, and
  `navigation_commands.rs` split those stable families by durable command
  surface rather than by incidental delivery order
- `src/cli/command_line.rs` owns the command-line assembly and parsing layer

## Command Execution

- `src/cli/commands/` owns the top-level operator workflows through
  `analyze.rs`, `artifact.rs`, `ingest.rs`, `run_pipeline.rs`, `synthetic.rs`,
  and the `validate/` and `diagnostics/` families
- `src/cli/commands/diagnostics/` owns diagnostics-facing workflows and report
  publication helpers
- `src/cli/commands/validate/` owns validation-specific command flows
- `src/cli/commands/run_pipeline_tests/` owns command-level harnesses that
  prove pipeline-command composition

## Runtime Setup And Support

- `src/cli/command_runtime.rs` plus `src/cli/command_runtime/` own
  runtime-environment setup, dataset inspection, acquisition reporting, and
  synthetic-reporting support needed during command execution
- `src/cli/command_support/` owns workflow-facing adapters for artifacts,
  capture windows, receiver artifacts, navigation outputs, and raw-IQ quality
- `src/cli/execution_support.rs` provides shared execution scaffolding

## Reporting

- `src/cli/report.rs` owns operator-facing output rendering

## Closest Proof

- `crates/bijux-gnss/src/main.rs`
- `crates/bijux-gnss/src/cli/command_catalog/`
- `crates/bijux-gnss/src/cli/commands/`
- `crates/bijux-gnss/src/cli/command_runtime.rs`
- `crates/bijux-gnss/src/cli/command_runtime/`
- `crates/bijux-gnss/src/cli/command_support/`
