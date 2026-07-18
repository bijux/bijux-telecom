---
title: Dependency Direction
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# Dependency Direction

`bijux-gnss-nav` should depend downward on shared contracts and signal-domain
identifiers, then present a higher scientific surface upward to runtime,
commands, and repository validation.

## Downward Edges

- `bijux-gnss-core` supplies shared time and artifact primitives that nav
  specializes rather than replaces
- `bijux-gnss-signal` supplies signal-domain identifiers and semantics where
  solver and correction behavior need them

## Upward Edges

- `bijux-gnss-receiver` consumes orbit, correction, and solution behavior to
  run live workflows
- `bijux-gnss` consumes the curated API to expose operator-visible commands
- `bijux-gnss-infra` consumes typed navigation truth when validating persisted
  evidence

## The Direction To Defend

- product parsing may depend on shared time and signal semantics, but not on
  runtime orchestration
- estimator code may depend on corrections and orbit state, but not on file
  layout or operator defaults
- public API composition may re-export types from internal families, but it
  should not expose runtime-only helpers by accident

## Warning Signs

- navigation parsers start depending on dataset discovery or path rules
- solver code gains knowledge of command defaults or run manifest semantics
- lower crates begin to depend on `nav` because shared contracts were left too
  high in the stack
