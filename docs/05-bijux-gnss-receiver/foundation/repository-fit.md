---
title: Repository Fit
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-18
---

# Repository Fit

`bijux-gnss-receiver` is the runtime center of `bijux-telecom`. It turns
configured sample streams, signal facts, navigation services, ports, and stage
logic into receiver evidence. It does not own command syntax, persisted
repository layout, standalone signal math, or navigation science.

## Runtime Role

```mermaid
flowchart LR
    command["command<br/>run request"]
    ports["ports<br/>sample source clock sinks"]
    receiver["receiver<br/>acquisition tracking observations"]
    signal["signal<br/>code and DSP substrate"]
    nav["nav<br/>science services"]
    artifacts["receiver artifacts<br/>in memory"]
    infra["infra<br/>persistence"]

    command --> receiver
    ports --> receiver
    signal --> receiver
    nav --> receiver
    receiver --> artifacts --> infra
```

Receiver owns the moment separate stage computations become one run with
ordered evidence. That ownership includes runtime configuration, stage
composition, source/sink ports, diagnostics, and receiver-owned artifacts before
infra persists them.

## Fit To Defend

| neighbor | receiver consumes | receiver refuses |
| --- | --- | --- |
| command | run requests, selected config, report needs | CLI shape and operator report policy |
| infra | dataset-resolved inputs and persisted-output handoff | run directory layout, manifests, artifact indexing |
| signal | code, carrier, replica, and DSP substrate | reusable signal facts and spreading-code generation |
| nav | solvers, correction services, and navigation products when enabled | standalone estimation law and product parsing ownership |
| core | shared records, units, diagnostics, and artifact payload shapes | changing shared field meaning locally |

## Reader Questions

- Is the issue acquisition, tracking, observation construction, runtime ports,
  diagnostics, synthetic runtime proof, or receiver artifacts? Stay here.
- Is the issue command syntax or report routing? Leave for `bijux-gnss`.
- Is the issue persisted run layout or dataset registry state? Leave for infra.
- Is the issue reusable code generation, signal identity, or DSP substrate?
  Leave for signal.
- Is the issue correction, orbit, estimator, PPP, RTK, or navigation format
  science? Leave for nav.

## First Proof Check

Inspect `crates/bijux-gnss-receiver/docs/BOUNDARY.md`,
`crates/bijux-gnss-receiver/docs/RUNTIME.md`,
`crates/bijux-gnss-receiver/docs/PIPELINE.md`,
`crates/bijux-gnss-receiver/docs/ARTIFACTS.md`,
`crates/bijux-gnss-receiver/src/engine/`,
`crates/bijux-gnss-receiver/src/pipeline/`, and
`crates/bijux-gnss-receiver/src/ports/`.
