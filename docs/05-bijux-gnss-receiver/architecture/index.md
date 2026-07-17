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

- open [Foundation](../foundation/) first if the real dispute is still about
  ownership rather than structure
- stay in this section when the question is where a runtime family belongs in
  code and which dependency direction is legitimate

## First Proof Check

- `crates/bijux-gnss-receiver/src/lib.rs`
- `crates/bijux-gnss-receiver/src/api.rs`
- `crates/bijux-gnss-receiver/docs/ARCHITECTURE.md`
