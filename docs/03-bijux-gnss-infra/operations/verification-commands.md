---
title: Verification Commands
audience: mixed
type: explanation
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
```

## Command Selection

- run `integration_overrides` when typed profile mutation or sweep behavior
  changes
- run `integration_guardrails` when dependency direction or workspace boundary
  pressure is involved

For dataset, run-layout, or validation-adapter changes that lack dedicated
crate-level tests today, review the crate-local docs and affected source
families explicitly rather than pretending a narrow automated proof already
exists.
