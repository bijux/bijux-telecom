---
title: Execution Model
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Execution Model

`bijux-gnss-receiver` is the package where configured components become one
runtime.

## Normal Flow

1. runtime configuration, sinks, and ports are assembled into a `Receiver`
2. sample frames are pulled from a source through the port boundary
3. acquisition produces candidate evidence and explainability artifacts
4. tracking turns accepted candidates into channel-state and tracking results
5. observations convert tracking results into observation artifacts and quality
   evidence
6. optional navigation and filtering produce navigation epochs when enabled
7. runtime artifacts, traces, and validation helpers summarize the run

## Important Architectural Distinction

The receiver crate owns execution ordering and runtime state, not the reusable
signal or navigation science used inside that order.

## Families With Separate Execution Logic

- acquisition has its own request planning, search-window, ranking, and
  refinement lifecycle
- tracking has its own channel lifecycle, reacquisition, and loop-state
  evolution
- observations have their own timing, smoothing, residual, and quality
  lifecycle
- synthetic execution can drive the same receiver boundary with scenario-backed
  sources and truth
