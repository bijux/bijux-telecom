---
title: Architecture
audience: mixed
type: index
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# Architecture

Open this section when the question is structural: where formats, orbits,
corrections, models, and estimation families live in code, and how the crate
stays broad without becoming a shapeless monolith.

## Structural Shape

`bijux-gnss-nav` is not one pipeline. It is a scientific package with explicit
subsystems: product interpretation, orbit and time reasoning, correction law,
and multiple estimation families that share physical assumptions but keep
separate responsibilities.

```mermaid
flowchart LR
    api["api.rs<br/>curated nav surface"]
    formats["formats/"]
    orbits["orbits/"]
    corrections["corrections/"]
    models["models/ and time/"]
    estimation["estimation/"]
    callers["receiver cli infra tests"]

    formats --> api
    orbits --> api
    corrections --> api
    models --> api
    estimation --> api
    api --> callers
```

## Read These First

- open [Foundation](../foundation/) first if the real dispute is still about
  ownership rather than structure
- stay in this section when the question is where a scientific family belongs
  in code and which dependency direction is legitimate

## First Proof Check

- `crates/bijux-gnss-nav/src/lib.rs`
- `crates/bijux-gnss-nav/src/formats.rs`
- `crates/bijux-gnss-nav/src/estimation.rs`
- `crates/bijux-gnss-nav/docs/ARCHITECTURE.md`
