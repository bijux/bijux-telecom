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

- [03-bijux-gnss-infra](../../03-bijux-gnss-infra/) explains persisted
  repository ownership below command workflows
- [05-bijux-gnss-receiver](../../05-bijux-gnss-receiver/) explains runtime
  execution below command workflows
- [04-bijux-gnss-nav](../../04-bijux-gnss-nav/) and
  [06-bijux-gnss-signal](../../06-bijux-gnss-signal/) explain the deeper
  science and signal owners this crate calls into

## The Fit To Defend

This crate should remain the place where operator workflows are explainable
without forcing readers to reconstruct a second CLI in lower crates.
