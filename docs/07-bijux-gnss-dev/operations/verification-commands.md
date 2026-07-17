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
```

## Matching Command To Change

- audit workflow changes:
  run `audit-allowlist`
- deviation workflow changes:
  run `deny-policy-deviations`
- nextest-roster or repository-shape changes:
  run the integration tests
- benchmark workflow changes:
  run the narrowest honest benchmark-related verification available in context
  and state clearly if full benchmark execution was skipped

## Bad Verification Pattern

Do not treat one passing guardrail test as proof for a changed governance
workflow. This crate owns several different maintainer contracts.
