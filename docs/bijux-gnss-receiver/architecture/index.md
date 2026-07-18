---
title: Receiver Architecture Guide
audience: mixed
type: index
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-18
---

# Receiver Architecture Guide

`bijux-gnss-receiver` owns staged runtime execution. The engine validates
configuration and composes effects; acquisition, tracking, observations, and
optional navigation adapters exchange typed state; artifacts and diagnostics
record what happened without choosing repository placement.

## Runtime Structure

```mermaid
flowchart LR
    config["validated receiver<br/>configuration"]
    ports["samples, clock,<br/>sinks, metrics"]
    engine["receiver engine"]
    acquisition["acquisition"]
    tracking["tracking"]
    observations["observations"]
    navigation["optional navigation"]
    artifacts["in-memory artifacts<br/>and diagnostics"]

    config --> engine
    ports --> engine
    engine --> acquisition --> tracking --> observations --> navigation
    acquisition --> artifacts
    tracking --> artifacts
    observations --> artifacts
    navigation --> artifacts
```

Every stage may enrich evidence, but no stage may erase uncertainty, ambiguity,
degradation, or refusal from the stage before it.

## Locate The Runtime Owner

| concern | architecture route | ownership rule |
| --- | --- | --- |
| Configuration, defaults, validation, runtime effects, support, or top-level composition | [Module map](module-map.md) | engine code owns receiver policy, not command defaults |
| Acquisition, tracking, observations, or optional navigation order | [Execution model](execution-model.md) | pipeline stages own handoff and lifecycle evidence |
| Samples, clocks, artifacts, metrics, tracing, or logs | [Integration seams](integration-seams.md) | effects cross explicit ports rather than hidden globals |
| Dependency on core, signal, navigation, infrastructure, or command | [Dependency direction](dependency-direction.md) | receiver composes lower science and is consumed by higher repository layers |
| Channel state, filter state, artifacts, or persisted runs | [State and persistence](state-and-persistence.md) | receiver owns in-memory runtime state; infrastructure owns durable placement |
| Input, acquisition, tracking, observation, or navigation failure | [Error model](error-model.md) | failure remains attributable to the earliest failing boundary |
| New stage, adapter, port, or artifact family | [Extensibility model](extensibility-model.md) | additions require durable runtime responsibility and evidence |
| Cross-stage coupling or imported repository policy | [Architecture risks](architecture-risks.md) | convenience must not blur scientific or persistence ownership |

## State Moves Forward, Evidence Fans Out

```mermaid
flowchart TD
    candidate["acquisition candidate"]
    channel["tracking channel state"]
    measurement["observation decision"]
    solution["optional navigation outcome"]
    acq_evidence["candidate evidence"]
    track_evidence["lock and transition evidence"]
    obs_evidence["quality and refusal evidence"]
    nav_evidence["solution or refusal evidence"]
    run["run artifacts"]

    candidate --> channel --> measurement --> solution
    candidate --> acq_evidence --> run
    channel --> track_evidence --> run
    measurement --> obs_evidence --> run
    solution --> nav_evidence --> run
```

The runtime path and evidence path are related but not identical. A rejected
candidate may never become a channel yet still belongs in acquisition evidence.
A refused observation may stop navigation input yet remain essential to
diagnosis.

## Effects Are Replaceable

Stage math must not open repository files, choose run directories, read
wall-clock time implicitly, or render command reports. Samples and time enter
through owned seams; artifacts, diagnostics, metrics, traces, and logs leave
through explicit runtime boundaries. Tests can replace those effects without
changing stage semantics.

## Optional Navigation Is An Adapter

The receiver decides when observations are ready for a navigation call and how
the resulting evidence joins a run. Navigation owns orbit, correction,
estimation, integrity, PPP, and RTK science. Feature-gating the adapter must not
change ownership or semantics of acquisition, tracking, or observations.

## Implementation Evidence

Use [code navigation](code-navigation.md) after identifying the concern. The
implementation authorities are the
[receiver engine](../../../crates/bijux-gnss-receiver/src/engine/mod.rs),
[pipeline boundary](../../../crates/bijux-gnss-receiver/src/pipeline/mod.rs),
[runtime ports](../../../crates/bijux-gnss-receiver/src/ports/mod.rs),
[sample adapters](../../../crates/bijux-gnss-receiver/src/io/mod.rs),
[artifact model](../../../crates/bijux-gnss-receiver/src/artifacts.rs), and
[simulation boundary](../../../crates/bijux-gnss-receiver/src/sim/mod.rs).

The [crate architecture](../../../crates/bijux-gnss-receiver/docs/ARCHITECTURE.md)
defines the complete runtime dataflow and dependency boundary.
