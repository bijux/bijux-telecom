---
title: Package Overview
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# Package Overview

`bijux-gnss` is the operator-facing command owner for the GNSS stack in
`bijux-telecom`.

## One-Sentence Role

This crate turns stable command vocabulary, runtime setup, and output rendering
into one public entrypoint over lower-level GNSS crates.

## What Readers Should Remember

- this crate owns commands and orchestration, not lower-level science
- reporting and workflow composition belong here
- the Rust facade exists for convenience, but it must stay thin and honest

## Major Command Families

- `src/main.rs` assembles the binary entrypoint from included CLI modules
- `src/cli/command_catalog/` owns stable command and argument shapes
- `src/cli/commands/` owns operator workflows such as ingest, synthetic,
  diagnostics, pipeline execution, analysis, and validation
- `src/cli/command_runtime/` owns runtime-environment setup, dataset
  inspection, and reporting support needed during command execution
- `src/cli/command_support/` owns workflow-facing adapters for artifacts,
  capture windows, navigation outputs, receiver artifacts, and raw-IQ support
- `src/cli/report.rs` owns operator-facing output rendering
- `src/lib.rs` owns the small package facade over lower-level crates

## Why This Package Is Thin On Purpose

The command boundary should expose one coherent product entrypoint without
duplicating signal, navigation, receiver, or persistence ownership. Thinness is
how this crate stays readable and durable.

## Closest Code Proof

- `crates/bijux-gnss/src/main.rs`
- `crates/bijux-gnss/src/cli/commands/`
- `crates/bijux-gnss/src/cli/command_runtime/`
- `crates/bijux-gnss/src/lib.rs`
