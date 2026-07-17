---
title: Repository Boundary Rules
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Repository Boundary Rules

This page replaces the old root-level architecture, layering, and codebase
rules notes. It records the structure rules maintainers are expected to defend.

## Dependency Direction

The primary product path flows from `bijux-gnss` into the lower crates. Command
entrypoints may depend on infra, receiver, nav, signal, and core. Receiver may
depend on nav, signal, and core. Signal and nav may depend on core. Shared
contracts do not depend back upward.

## Public Boundary Rule

- each crate exposes a curated public facade rather than re-exporting its
  entire source tree
- internal modules stay private unless a real cross-crate contract exists
- modules that only forward exports without owning behavior should be removed
  instead of preserved as decorative structure

## Structure Discipline

- path depth should stay shallow enough that ownership is readable from the
  path itself
- semantic file names beat history-driven names
- module trees should group by durable responsibility, not by temporary delivery
  sequence

## Reader Rule

When a proposed file, module, or crate name needs a long explanation to defend
why it exists, the structure is usually not clean enough yet.

## First Proof Check

Inspect `crates/bijux-gnss-dev/docs/BOUNDARY.md`,
`crates/bijux-gnss-dev/docs/PUBLIC_API.md`,
`crates/bijux-gnss-dev/tests/integration_guardrails.rs`, and
`crates/bijux-gnss-dev/src/main.rs` to confirm the repository boundary rules
described here still match the enforced maintainer crate shape.
