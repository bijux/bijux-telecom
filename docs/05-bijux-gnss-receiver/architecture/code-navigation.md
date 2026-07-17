---
title: Code Navigation
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Code Navigation

Use this page when you know the runtime question but not the owning file.

## Start From The Question

- "How is the receiver configured or launched?":
  `src/engine/`, especially `receiver_config.rs`, `receiver_config_validation.rs`,
  `runtime.rs`, and `engine.rs`
- "How is a stage executed or handed off?":
  `src/pipeline/` and the matching acquisition, tracking, observations, or
  navigation family
- "How do samples, clocks, or sinks enter the runtime?":
  `src/ports/` and `src/io/`
- "How are run artifacts or validation reports built?":
  `src/artifacts.rs`, `src/reference_validation.rs`,
  `src/validation_helpers.rs`, and `src/validation_report.rs`
- "How does the synthetic runtime prove this behavior?":
  `src/sim/synthetic/`

## Start From A Public Export

- begin in `src/api.rs`
- locate the re-exported family
- move into the owning submodule only after confirming the export is public on
  purpose

## Start From A Test Failure

- acquisition failures usually map to
  `crates/bijux-gnss-receiver/tests/integration_acquisition_*` or
  `src/pipeline/acquisition/tests/`
- tracking failures usually map to
  `crates/bijux-gnss-receiver/tests/integration_tracking_*` or
  `src/pipeline/tracking/tests/`
- observation failures usually map to
  `crates/bijux-gnss-receiver/tests/integration_observations_*` or
  `src/pipeline/observations/tests/`
- validation or synthetic failures often map to
  `src/validation_report/tests/` or `src/sim/synthetic/tests/`
