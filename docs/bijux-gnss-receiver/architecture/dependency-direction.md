---
title: Dependency Direction
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Dependency Direction

`bijux-gnss-receiver` should depend downward on shared contracts and scientific
owners, then present a higher runtime surface upward to commands and
repository-facing tooling.

## Downward Edges

- `bijux-gnss-core` supplies shared time, artifacts, diagnostics, and
  observation contracts
- `bijux-gnss-signal` supplies signal primitives and reusable lower-level stage
  logic inputs
- `bijux-gnss-nav` supplies navigation-stage science and validation-adjacent
  types when the feature is enabled

## Upward Edges

- `bijux-gnss` consumes the receiver boundary to run operator workflows
- `bijux-gnss-infra` consumes receiver artifacts and runtime-side validation
  outputs before persisting and inspecting them

## The Direction To Defend

- runtime configuration may select stage behavior, but should not redefine
  lower-level signal or navigation science
- ports may carry paths or run identifiers as runtime controls, but should not
  own persisted repository semantics
- public API composition may re-export lower owners for convenience, but should
  not blur ownership of those families

## Warning Signs

- stage modules start owning generic signal primitives
- runtime config or artifacts gain command-only wording or formatting policy
- receiver-owned validation begins depending on repository layout assumptions
