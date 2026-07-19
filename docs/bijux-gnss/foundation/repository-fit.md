---
title: Repository Fit
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# Repository Fit

`bijux-gnss` is the public command face of the repository. It is not the
runtime center and not the persistence layer.

## Why The Repository Needs This Layer

- the repository needs one installable operator boundary rather than exposing
  every lower subsystem directly
- commands should stay coherent even while lower-level crates evolve
- operators should not have to understand crate boundaries just to invoke a
  workflow

## How It Fits With Neighboring Handbooks

- [Infra handbook](../../bijux-gnss-infra/index.md) explains persisted
  repository ownership below command workflows
- [Receiver handbook](../../bijux-gnss-receiver/index.md) explains runtime
  execution below command workflows
- [Navigation handbook](../../bijux-gnss-nav/index.md) and
  [Signal handbook](../../bijux-gnss-signal/index.md) explain the deeper
  science and signal owners this crate calls into

## The Fit To Defend

This crate should remain the place where operator workflows are explainable
without forcing readers to reconstruct a second CLI in lower crates.
