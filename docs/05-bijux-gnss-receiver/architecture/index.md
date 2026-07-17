---
title: Architecture
audience: mixed
type: index
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Architecture

Open this section when the question is structural: where engine, pipeline,
ports, artifacts, validation, and simulation live in code, and how the crate
stays broad without becoming a shapeless runtime bucket.

## Structural Shape

```mermaid
flowchart LR
    api["api.rs<br/>curated receiver surface"]
    engine["engine/"]
    pipeline["pipeline/"]
    ports["ports/ and io/"]
    artifacts["artifacts and validation"]
    sim["sim/"]
    callers["gnss infra tests"]

    engine --> api
    pipeline --> api
    ports --> api
    artifacts --> api
    sim --> api
    api --> callers
```

## Read These First

- open [Module Map](module-map.md) first when you need the fastest route from
  a runtime concern to the owning code area
- open [Dependency Direction](dependency-direction.md) when the question is
  whether receiver runtime is aggregating lower-level behavior honestly
- open [Integration Seams](integration-seams.md) when a change seems to pull
  command, repository, or science policy inward

## Pages In This Section

- [Module Map](module-map.md)
- [Dependency Direction](dependency-direction.md)
- [Execution Model](execution-model.md)
- [State And Persistence](state-and-persistence.md)
- [Integration Seams](integration-seams.md)
- [Error Model](error-model.md)
- [Extensibility Model](extensibility-model.md)
- [Code Navigation](code-navigation.md)
- [Architecture Risks](architecture-risks.md)

## First Proof Check

- `crates/bijux-gnss-receiver/src/lib.rs`
- `crates/bijux-gnss-receiver/src/api.rs`
- `crates/bijux-gnss-receiver/docs/ARCHITECTURE.md`

## Leave This Section When

- leave for [Foundation](../foundation/) when the real dispute is still about
  ownership rather than structure
- leave for [Interfaces](../interfaces/) when the structural question is
  already about public contract shape
- leave for [Quality](../quality/) when the structure is clear and the next
  question is proof sufficiency
