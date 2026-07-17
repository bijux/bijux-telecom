---
title: Verification Commands
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Verification Commands

Run the smallest honest proof set that matches the changed maintainer surface.

## Common Commands

```sh
cargo test -p bijux-gnss-dev --test integration_guardrails
cargo test -p bijux-gnss-dev --test integration_nextest_suite_selection
cargo run -p bijux-gnss-dev -- audit-allowlist
cargo run -p bijux-gnss-dev -- deny-policy-deviations
cargo run -p bijux-gnss-dev -- audit-ignore-args
```

## Matching Command To Change

- audit workflow changes:
  run `audit-allowlist`
- deviation workflow changes:
  run `deny-policy-deviations`
- derived audit-ignore behavior changes:
  run `audit-ignore-args`
- nextest-roster or repository-shape changes:
  run the integration tests
- benchmark workflow changes:
  run `bijux-gnss-dev bench-compare` or `make bench-compare` when the current
  context can tolerate benchmark cost, and state clearly when full benchmark
  execution was skipped

Treat `crates/bijux-gnss-dev/docs/TESTS.md` as the proof map when a change
touches command validation, roster governance, and benchmark evidence routing
at the same time.

## Bad Verification Pattern

Do not treat one passing guardrail test as proof for a changed governance
workflow. This crate owns several different maintainer contracts.

## First Proof Check

Use `crates/bijux-gnss-dev/docs/TESTS.md`,
`crates/bijux-gnss-dev/docs/WORKFLOWS.md`,
`crates/bijux-gnss-dev/docs/GOVERNANCE_FILES.md`, and
`crates/bijux-gnss-dev/docs/OUTPUTS.md` as the verification map. Then inspect
the changed command path in `crates/bijux-gnss-dev/src/main.rs` so command
choice stays tied to the real maintainer workflow.
