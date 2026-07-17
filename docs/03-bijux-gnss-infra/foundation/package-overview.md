---
title: Package Overview
audience: mixed
type: explanation
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Package Overview

`bijux-gnss-infra` exists to make repository-facing GNSS state typed,
repeatable, and inspectable.

## Role Model

```mermaid
flowchart LR
    repo["datasets configs artifacts histories"]
    infra["bijux-gnss-infra"]
    consumers["gnss receiver nav testkit maintainers"]

    repo --> infra --> consumers
```

If the repository needs one stable interpretation of datasets, run identity,
artifact persistence context, profile overrides, or provenance hashing, this is
usually the crate that should own it.

## Boundary Verdict

If the work makes repository state more typed, reproducible, or auditable
without becoming runtime or solver policy, it belongs here. If it starts
deciding signal processing, navigation estimation, receiver scheduling, or
command UX, it has crossed the boundary.

## What This Package Makes Possible

- commands and tests can rely on one dataset registry interpretation
- persisted run footprints stay stable after the command that produced them is
  gone
- experiment variation stays typed instead of becoming shell string handling
- artifact inspection can happen after execution without re-entering runtime
  code

## Tempting Mistakes

- putting receiver defaults into overrides because they are "config related"
- storing repository layout policy inside artifact payload definitions
- making command semantics responsible for dataset resolution rules that should
  be shared everywhere

## First Proof Check

- `crates/bijux-gnss-infra/src/datasets/registry.rs`
- `crates/bijux-gnss-infra/src/run_layout/`
- `crates/bijux-gnss-infra/src/sweep.rs`
- `crates/bijux-gnss-infra/src/artifact_inspection/`
