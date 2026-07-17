---
title: Verification Commands
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Verification Commands

These are the core crate’s most useful narrow checks from the repository root.

```sh
cargo test -p bijux-gnss-core --test public_api_guardrail
cargo test -p bijux-gnss-core --test nav_artifact_validation
cargo test -p bijux-gnss-core --test tracking_artifact_validation
cargo test -p bijux-gnss-core --test prop_timekeeping
cargo test -p bijux-gnss-core --test integration_guardrails
```

## Command Selection

- run `public_api_guardrail` when `api.rs` or public exports move
- run artifact validation tests when serialized payload meaning or validators
  change
- run `prop_timekeeping` when time conversion or sample-trace behavior changes
- run `integration_guardrails` when dependency direction or workspace boundary
  pressure is involved
- treat `crates/bijux-gnss-core/docs/TESTS.md` as the reference map when a
  contract family change needs a narrower proof route than this page lists

## First Proof Check

Use `crates/bijux-gnss-core/docs/TESTS.md`,
`crates/bijux-gnss-core/docs/PUBLIC_API.md`, and
`crates/bijux-gnss-core/docs/CONTRACTS.md` as the verification map. Then
inspect the changed source family under `crates/bijux-gnss-core/src/` so the
proof set stays aligned with the actual contract family that moved.
