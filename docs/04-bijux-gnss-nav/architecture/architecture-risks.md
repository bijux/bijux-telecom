---
title: Architecture Risks
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# Architecture Risks

This page records the main structural risks in `bijux-gnss-nav`.

## Main Risks

- the crate is broad enough that convenience exports can slowly erase internal
  boundaries
- runtime-neutral helpers can be mistaken for permission to own runtime
  orchestration
- solver-local policy can leak into the public API because downstream crates
  genuinely consume many surfaces
- precise-product parsing and validation pressures can tempt repository logic
  inward
- local matrix and model helpers can drift toward unreviewed generic utility
  ownership if not kept tied to navigation use

## What To Watch In Review

- whether a new path is scientific or merely caller-convenient
- whether a public export is durable or only temporarily useful
- whether a new family belongs under an existing subsystem rather than as a new
  peer at the top level
