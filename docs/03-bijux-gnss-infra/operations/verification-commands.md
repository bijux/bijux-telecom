---
title: Verification Commands
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Verification Commands

These are the most useful narrow checks from the repository root for infra
changes.

```sh
cargo test -p bijux-gnss-infra --test integration_overrides
cargo test -p bijux-gnss-infra --test integration_guardrails
cargo test -p bijux-gnss-infra
```

## Command Selection

- run `integration_overrides` when typed profile mutation or sweep behavior
  changes
- run `integration_guardrails` when dependency direction or workspace boundary
  pressure is involved
- run the full crate test command when dataset parsing, raw-IQ metadata,
  provenance hashing, artifact inspection, or run-layout provenance changes
  touch module-level tests rather than integration targets

Treat `crates/bijux-gnss-infra/docs/TESTS.md` as the reference map when a
change spans datasets, run layout, overrides, or validation and you need the
best current proof route. Validation-adapter changes still require explicit
review of `docs/VALIDATION.md`, `src/api.rs`, and `src/validate_reference.rs`
because there is not yet a dedicated integration target for that boundary.
