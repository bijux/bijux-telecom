---
title: Package Overview
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-18
---

# Package Overview

`bijux-gnss-receiver` owns receiver runtime execution: configuration becomes a
running receiver, sample frames enter through ports, stages produce evidence,
and runtime artifacts leave the crate before repository persistence takes over.

This crate is large because the receiver boundary is large. Acquisition,
tracking, observations, optional navigation handoff, runtime sinks, simulation,
and receiver-side validation must agree on one execution story.

## Runtime Chain

```mermaid
flowchart LR
    config["receiver config"]
    ports["source, clock,<br/>logger, trace, metrics"]
    acquisition["acquisition"]
    tracking["tracking"]
    observations["observations"]
    navigation["optional navigation"]
    artifacts["run artifacts<br/>validation reports"]

    config --> ports --> acquisition --> tracking --> observations --> navigation --> artifacts
    observations --> artifacts
```

## Owned Families

| family | owns | first proof |
| --- | --- | --- |
| engine | runtime config, validation, defaults, logging, metrics, support matrix, receiver composition | `crates/bijux-gnss-receiver/src/engine/` |
| ports and I/O | sample sources, artifact sinks, clocks, runtime effect seams | `crates/bijux-gnss-receiver/src/ports/`, `crates/bijux-gnss-receiver/src/io/` |
| acquisition | request planning, search windows, candidates, ranking, explainability, handoff evidence | `crates/bijux-gnss-receiver/src/pipeline/acquisition.rs`, `crates/bijux-gnss-receiver/src/pipeline/acquisition/` |
| tracking | channel lifecycle, lock evidence, loop state, reacquisition, sample-rate diagnostics | `crates/bijux-gnss-receiver/src/pipeline/tracking.rs`, `crates/bijux-gnss-receiver/src/pipeline/tracking/` |
| observations | pseudorange, carrier phase, smoothing, residuals, quality, epoch manifests | `crates/bijux-gnss-receiver/src/pipeline/observations.rs`, `crates/bijux-gnss-receiver/src/pipeline/observations/` |
| navigation handoff | receiver-owned calls into navigation solving and filtering when enabled | `crates/bijux-gnss-receiver/src/pipeline/navigation.rs`, `crates/bijux-gnss-receiver/src/pipeline/navigation_filter.rs` |
| simulation | synthetic sources, truth, stage accuracy, scenario validation | `crates/bijux-gnss-receiver/src/sim/` |
| validation reports | receiver-side accuracy, consistency, covariance, and reference checks | `crates/bijux-gnss-receiver/src/validation_report.rs`, `crates/bijux-gnss-receiver/src/reference_validation.rs` |

## Reader Rules

- Use this handbook for execution order, state handoff, runtime diagnostics,
  and in-memory receiver artifacts.
- Leave for `bijux-gnss-signal` when the question is reusable code generation,
  signal catalog truth, sample math, front-end filtering, NCO behavior, or DSP
  discriminators outside one receiver run.
- Leave for `bijux-gnss-nav` when the question is navigation science rather
  than receiver-side invocation of that science.
- Leave for `bijux-gnss-infra` when runtime artifacts are named, persisted,
  indexed, compared across run directories, or tied to dataset registry state.
- Leave for `bijux-gnss-core` when the question is shared record meaning,
  observation fields, diagnostic codes, units, or artifact envelopes.

## Failure Reading

When a receiver test fails, first identify which stage emitted the wrong
evidence. Do not flatten every receiver failure into “pipeline”:

- acquisition failures usually involve signal model selection, code phase,
  Doppler search, candidate ranking, ambiguity, or explainability artifacts
- tracking failures usually involve lock state, loop bandwidth, Doppler ramp,
  reacquisition, fade handling, or channel diagnostics
- observation failures usually involve timestamping, pseudorange, carrier
  phase, smoothing, residuals, or quality classification
- navigation failures at this boundary usually involve receiver-to-nav handoff
  or filter configuration, not standalone navigation model law

## First Proof Check

Inspect `crates/bijux-gnss-receiver/README.md`,
`crates/bijux-gnss-receiver/docs/PIPELINE.md`,
`crates/bijux-gnss-receiver/docs/RUNTIME.md`,
`crates/bijux-gnss-receiver/docs/ARTIFACTS.md`, the relevant `src/pipeline/`
stage, and the integration tests named in the crate README.
