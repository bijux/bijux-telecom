---
title: Stage Contracts
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Stage Contracts

Stage contracts define how the runtime exposes acquisition, tracking,
observation, and optional navigation behavior.

## Owned Stage Families

- acquisition engine and acquisition-assistance helpers
- tracking engine and channel-state or tracking-artifact types
- observation builders, residual reports, and measurement-quality reports
- optional `Navigation` and `NavigationFilter` receiver-owned adapters over
  nav-owned science
- `StepReport` and `StepStats` as handoff and report helpers

## Boundary Rule

These contracts are about runtime composition and handoff. The reusable signal
or navigation science used inside them still belongs to lower crates.

## Closest Proof

- `crates/bijux-gnss-receiver/src/pipeline/`
- `crates/bijux-gnss-receiver/docs/PIPELINE.md`
