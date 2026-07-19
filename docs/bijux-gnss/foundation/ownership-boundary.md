---
title: Ownership Boundary
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# Ownership Boundary

`bijux-gnss` sits at the top of the GNSS Rust package stack and serves as the
operator command boundary.

## Downstream Crates It May Compose

- `bijux-gnss-core` for shared contracts and schemas
- `bijux-gnss-signal` for signal primitives and command-facing diagnostics
- `bijux-gnss-nav` for navigation decode, validation, and solver-backed flows
- `bijux-gnss-receiver` for runtime execution and in-memory artifacts
- `bijux-gnss-infra` for dataset and repository-facing workflows

## Boundary Tests

- if a concern starts from a stable command name, flag, or report shape, it
  can belong here
- if a concern starts from how a receiver stage executes internally, it does
  not belong here
- if a concern starts from repository persistence rules, it does not belong
  here
- if a concern changes which lower-level workflow a command wires together, it
  does belong here

## Where Boundary Drift Usually Starts

- command handlers begin re-implementing lower-level science instead of calling
  owned APIs
- report rendering starts carrying repository persistence policy
- the facade grows one-off helpers that really belong in a lower crate
