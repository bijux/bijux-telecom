---
title: Module Map
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Module Map

This page maps the main runtime families in `bijux-gnss-receiver` to the code
that owns them.

## Top-Level Surface

- `src/api.rs` is the curated downstream runtime surface
- `src/lib.rs` states the durable crate map: `engine`, `io`, `pipeline`,
  `ports`, `artifacts`, runtime-side validation, and `sim`

## Runtime Composition

- `src/engine/receiver_config.rs`, `receiver_config_defaults.rs`, and
  `receiver_config_validation.rs` plus the nested
  `src/engine/receiver_config/` family own configuration shape, defaults, and
  schema validation
- `src/engine/runtime.rs` owns runtime sinks and side-effectful controls
- `src/engine/engine.rs` and `src/engine/receiver.rs` own top-level execution
  entrypoints
- `src/engine/logging.rs`, `src/engine/metrics.rs`,
  `src/engine/diagnostics.rs`, `src/engine/signal_selection.rs`, and
  `src/engine/support_matrix.rs` own runtime instrumentation, signal
  selection, and support reporting

## Stage Pipeline

- `src/pipeline/acquisition/` owns acquisition execution, ranking,
  refinement, uncertainty, and wrong-PRN or front-end rejection behavior
- `src/pipeline/tracking/` owns channel lifecycle, loop state, reacquisition,
  session artifacts, and tracking evidence
- `src/pipeline/observations/` owns observation construction, smoothing,
  residuals, quality, timing, and lock-state interpretation
- `src/pipeline/navigation.rs` and `navigation_filter.rs` own receiver-side
  navigation-stage adapters over nav-owned science
- `src/pipeline/acquisition_assistance.rs`, `acquisition_components.rs`,
  `acquisition_symbol_hypotheses.rs`, `doppler.rs`, `hatch.rs`, and
  `signal_capabilities.rs` own cross-stage helpers that remain receiver-owned
- `src/pipeline/observation_validation.rs` owns carrier-smoothed code
  validation at the receiver boundary

## Runtime Seams

- `src/ports/` owns source, sink, and clock traits
- `src/io/data.rs` owns in-memory and file-backed sample-source adapters

## Receiver-Boundary Outputs

- `src/artifacts.rs` owns `RunArtifacts` helpers
- `src/reference_validation.rs` owns runtime-side reference alignment and
  comparison helpers
- `src/validation_report.rs`, `src/validation_helpers.rs`, and
  `src/validation_report/tests/` own validation-report assembly and supporting
  evidence
- `src/covariance_realism.rs` owns covariance realism summaries when `nav` is
  enabled

## Synthetic Runtime Proof

- `src/sim/synthetic/` owns scenario execution, signal generation, acquisition
  validation, artifact validation, stage accuracy, and runtime sensitivity
  profiles
