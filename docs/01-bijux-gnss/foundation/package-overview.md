---
title: Package Overview
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-18
---

# Package Overview

`bijux-gnss` owns the operator-facing command boundary. If a human starts by
typing `bijux gnss ...`, this crate is the first owner even when the command
later calls receiver, signal, navigation, core, or infrastructure code.

The crate should stay thin, but not vague. It owns the shape of operator work:
argument names, command grouping, runtime setup, report rendering, and the
handoff from a command route to the crate that owns the deeper GNSS behavior.

## Command Handoff

```mermaid
flowchart LR
    operator["operator command"]
    catalog["command catalog<br/>argument shape"]
    runtime["command runtime<br/>datasets, configs, reports"]
    lower["owning GNSS crate<br/>receiver, signal, nav, infra, core"]
    output["operator output<br/>table, JSON, artifacts"]

    operator --> catalog --> runtime --> lower --> output
```

## Owned Surfaces

| surface | owns | first proof |
| --- | --- | --- |
| binary entrypoint | `bijux` and the `gnss` command tree | binary entrypoint and command-line source |
| command catalog | stable subcommand and argument families | command catalog source |
| command handlers | operator workflow routing and lower-crate calls | command handler source |
| runtime setup | config loading, dataset resolution, synthetic reporting, acquisition reporting | command runtime source |
| command support | artifact loading, capture windows, raw-IQ quality, navigation output adapters | command support source |
| operator reports | table and JSON output shape | report source |
| Rust facade | convenience re-exports over lower crates | facade source |

## Reader Rules

- Start here for command names, arguments, report shape, and operator workflow
  composition.
- Leave for `bijux-gnss-receiver` when the question is stage behavior,
  runtime artifacts, acquisition, tracking, observations, or receiver-side
  validation.
- Leave for `bijux-gnss-infra` when the question is dataset registry meaning,
  sidecar metadata, run layout, overrides, or persisted provenance.
- Leave for `bijux-gnss-nav` when the question is ephemeris decoding, precise
  products, corrections, PVT, PPP, RTK, or navigation-time behavior.
- Leave for `bijux-gnss-signal` when the command exposes spreading codes,
  signal identity, sample contracts, raw-IQ interpretation, or DSP primitives.
- Leave for `bijux-gnss-core` when the disagreement is shared record meaning:
  IDs, units, time, observations, navigation solutions, diagnostics, or
  artifact envelopes.

## Boundary Tests

A command change belongs in this crate when it changes how an operator asks for
work or reads the result. It does not belong here when it changes the science
or persistence semantics behind that route.

| change | owner |
| --- | --- |
| rename or add a CLI argument | `bijux-gnss` |
| change how a dataset id resolves | `bijux-gnss-infra` |
| change acquisition ranking, tracking state, or observation construction | `bijux-gnss-receiver` |
| change code generation or DSP behavior | `bijux-gnss-signal` |
| change orbit, correction, PVT, PPP, or RTK logic | `bijux-gnss-nav` |
| change serialized observation or diagnostic meaning shared across crates | `bijux-gnss-core` |

## First Proof Check

Inspect the [command crate README](../../../crates/bijux-gnss/README.md),
[command guide](../../../crates/bijux-gnss/docs/COMMANDS.md),
[public API](../../../crates/bijux-gnss/docs/PUBLIC_API.md), command source,
and the integration tests named in the crate README. A package overview is
stale if it cannot point from an operator route to the exact lower owner that
proves the route.
