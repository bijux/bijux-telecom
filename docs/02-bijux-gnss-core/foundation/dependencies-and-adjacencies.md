---
title: Dependencies and Adjacencies
audience: mixed
type: explanation
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Dependencies and Adjacencies

`bijux-gnss-core` should have simple dependency posture and very explicit
adjacencies.

## Dependency Direction

- downstream crates may depend on `bijux-gnss-core`
- `bijux-gnss-core` must not depend on `bijux-gnss-signal`,
  `bijux-gnss-nav`, `bijux-gnss-receiver`, `bijux-gnss-infra`, or
  `bijux-gnss`

## Adjacent Package Pressure

- `signal` pulls on core when it needs reusable identifiers and record meaning
- `nav` pulls on core when it needs observation and solution contracts
- `receiver` pulls on core when it needs exchangeable stage records and artifact
  payloads
- `infra` pulls on core when it validates or inspects persisted outputs

## Dependency Smells

- introducing a dependency because one local helper would be convenient
- importing behavior from a higher layer instead of defining a clean contract
- making core depend on one owner’s implementation because it happens to be the
  first one written

## First Proof Check

- `crates/bijux-gnss-core/Cargo.toml`
- `crates/bijux-gnss-core/docs/ARCHITECTURE.md`
- `crates/bijux-gnss-core/tests/integration_guardrails.rs`
