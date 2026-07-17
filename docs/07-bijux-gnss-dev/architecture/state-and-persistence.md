---
title: State And Persistence
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# State And Persistence

This crate holds short-lived command state and writes only reviewed maintenance
evidence.

## Allowed State

- parsed CLI command state
- temporary in-memory validation results
- benchmark snapshots and comparison maps held during one command execution

## Allowed Persistence

- `artifacts/benchmarks.txt` as raw bencher output
- `benchmarks/bencher_current.txt` as the normalized current snapshot written
  by `bench-compare`
- reads against `benchmarks/bencher_baseline.txt` when a checked-in baseline is
  present

## Forbidden Persistence

- hidden temp files outside governed locations
- product artifacts that belong to runtime or infrastructure crates
- long-lived repository data with no documented maintainer workflow owner
- writes from validation or derivation commands that should have remained
  read-only
