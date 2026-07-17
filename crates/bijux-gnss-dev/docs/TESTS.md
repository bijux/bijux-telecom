# Tests

## Entry points

- `tests/integration_guardrails.rs` verifies this crate still satisfies workspace guardrail policy.

## What is intentionally not here

This crate currently has no large standalone test matrix because most of its behavior is repository
file validation with direct command implementations in `src/main.rs`. The important contract today
is that the command surface stays narrow, repository-owned, and policy-compliant.

Command execution is still part of verification for this crate because the main product surface is
the binary itself rather than a Rust library API.

## Verification

Useful validation commands from the repository root:

```sh
cargo test -p bijux-gnss-dev --test integration_guardrails
cargo run -p bijux-gnss-dev -- audit-allowlist
cargo run -p bijux-gnss-dev -- deny-policy-deviations
```
