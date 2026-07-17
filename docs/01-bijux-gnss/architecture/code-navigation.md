---
title: Code Navigation
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# Code Navigation

Use this page when you know the command-boundary question but not the owning
file.

## Start From The Question

- "How is this command or flag defined?":
  `src/cli/command_catalog/` and `src/cli/command_line.rs`
- "Which workflow handles this command?":
  `src/cli/commands/`
- "How does the command prepare inputs or environment?":
  `src/cli/command_runtime.rs` and `src/cli/command_runtime/`
- "How does this command load or reshape lower-owner outputs?":
  `src/cli/command_support/`
- "How is the output rendered?":
  `src/cli/report.rs`
- "Why is this Rust surface exported from the top package?":
  `src/lib.rs`

## Start From A Public Surface

- begin in `src/main.rs` for binary concerns
- begin in `src/lib.rs` for Rust-package facade concerns
- move into CLI submodules only after confirming the owning command family

## Start From A Test Failure

- command and workflow failures usually map to the matching
  `crates/bijux-gnss/tests/integration_*.rs` family, especially
  `integration_validate_*`, `integration_nav_decode.rs`, and
  `integration_export_synthetic_iq.rs`
- command-support regressions often map to
  `crates/bijux-gnss/src/cli/command_support/tests/`
- pipeline-command composition regressions often map to
  `crates/bijux-gnss/src/cli/commands/run_pipeline_tests/`
