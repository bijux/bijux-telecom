---
title: Change Validation
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Change Validation

Choose the smallest honest proof set that exercises the changed maintainer
workflow.

## Minimum Validation By Change Type

- audit workflow change:
  run `audit-allowlist`
- deviation workflow change:
  run `deny-policy-deviations`
- roster or repository-structure policy change:
  run the integration tests
- benchmark workflow change:
  run the narrowest honest benchmark-related proof available and state clearly
  if full benchmark execution was skipped

## Bad Validation Patterns

- relying on one unrelated integration test for a changed governed-file rule
- changing evidence outputs without checking the documented output contract
- claiming benchmark workflow safety without stating whether heavy execution was
  actually run

## First Proof Check

Use `crates/bijux-gnss-dev/docs/TESTS.md`,
`crates/bijux-gnss-dev/docs/WORKFLOWS.md`, and
`crates/bijux-gnss-dev/docs/OUTPUTS.md` as the validation map. Then inspect
the matching command path in `crates/bijux-gnss-dev/src/main.rs` before
declaring the proof set complete.
