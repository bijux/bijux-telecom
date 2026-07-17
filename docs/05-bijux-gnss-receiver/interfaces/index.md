---
title: Interfaces
audience: mixed
type: index
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Interfaces

Open this section when the question is contractual: which runtime, stage, port,
artifact, validation, and simulation surfaces are safe for another crate or
tool to rely on.

## Contract Surface

```mermaid
flowchart LR
    caller["caller or tool"]
    api["bijux_gnss_receiver::api"]
    runtime["runtime contracts"]
    stages["stage contracts"]
    ports["port contracts"]
    artifacts["artifact contracts"]
    validation["validation and sim contracts"]

    caller --> api
    api --> runtime
    api --> stages
    api --> ports
    api --> artifacts
    api --> validation
```

## Read These First

- open [Foundation](../foundation/) first if the question is whether a public
  surface belongs in receiver at all
- stay in this section when the question is whether an export, trait, or
  runtime record deserves a durable public promise

## First Proof Check

- `crates/bijux-gnss-receiver/src/api.rs`
- `crates/bijux-gnss-receiver/API.md`
- `crates/bijux-gnss-receiver/docs/PUBLIC_API.md`
