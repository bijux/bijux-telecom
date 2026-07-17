---
title: Extensibility Model
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Extensibility Model

The crate grows by adding new durable signal families or reusable DSP surfaces,
not by inserting generic extension hooks everywhere.

## Honest Extension Paths

- add a new constellation or band family under `codes/` when the repository
  gains durable support for that signal
- add a new reusable DSP primitive when it remains valuable outside one runtime
- extend `catalog.rs` when the supported-signal registry truly expands
- extend `raw_iq.rs`, `samples.rs`, or `obs_validation.rs` when a stable signal
  contract needs a new field or report dimension

## Suspicious Extension Paths

- adding a helper that exists only to patch one receiver workflow
- creating a generic bucket module instead of naming the actual signal family
- exporting internal lookup tables that should remain implementation detail

## Compatibility Discipline

Every extension should answer three questions before it lands:

- is this a durable signal owner, not a temporary convenience
- is the name organized by signal meaning or mathematical role
- can the proof surface demonstrate the new behavior independently of one
  higher-level workflow
