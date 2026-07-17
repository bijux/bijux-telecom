---
title: Module Map
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Module Map

The crate currently keeps its owned workflows inside one binary source file on
purpose.

## Top-Level Structure

- `src/main.rs` owns CLI parsing, subcommand inventory, command dispatch, file
  validation logic, benchmark execution, snapshot writing, and baseline
  comparison
- `tests/integration_guardrails.rs` proves the crate still fits workspace
  guardrail policy
- `tests/integration_nextest_suite_selection.rs` proves slow-test roster
  integrity against the repository source tree

## Command Regions Inside `main.rs`

- CLI definitions:
  `Cli` and `Commands`
- governed-file validation:
  `run_audit_allowlist_check` and
  `run_deny_policy_deviations_check`
- derived-command adapter:
  `run_audit_ignore_args`
- benchmark governance:
  `run_bench_compare`, `run_bench`, `write_current_snapshot`, and
  `compare_baseline`
- shared validation helpers:
  date and ID shape checks

## Why One File Is Acceptable Today

The command surface is still narrow and the ownership unit is the binary
itself. If the crate grows, it should split by durable workflow family such as
audit governance or benchmark governance, not by temporary sequencing.
