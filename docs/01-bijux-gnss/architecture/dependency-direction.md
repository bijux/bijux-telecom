---
title: Dependency Direction
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# Dependency Direction

`bijux-gnss` sits at the top of the package stack. It should depend downward on
runtime, repository, signal, navigation, and shared owners, then expose a
stable operator boundary upward.

## Downward Edges

- `bijux-gnss-core` for shared contracts and schemas
- `bijux-gnss-signal` for signal-facing command workflows
- `bijux-gnss-receiver` for runtime execution and in-memory artifacts
- `bijux-gnss-infra` for repository-facing workflows and validation support
- `bijux-gnss-nav` for navigation decode and validation flows when enabled

## Upward Contract

- operators and automation should touch the GNSS stack primarily through this
  crate's binary surface
- Rust consumers may use the small facade when they need one package-level
  entrypoint, but that facade must stay thin

## The Direction To Defend

- command handlers may orchestrate lower crates, but should not re-own their
  deeper behavior
- report rendering may summarize lower-level outcomes, but should not redefine
  their scientific meaning
- the facade may re-export lower-level crates, but should not become a second
  mixed-responsibility API layer

## Warning Signs

- a lower-level algorithm is copied into a command module
- commands begin to depend on repository path conventions instead of infra APIs
- facade exports grow faster than the command surface itself
