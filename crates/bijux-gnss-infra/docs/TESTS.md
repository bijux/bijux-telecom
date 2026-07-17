# Tests

## Entry points

- `tests/integration_overrides.rs` validates infrastructure-owned override behavior.
- `tests/integration_guardrails.rs` keeps the crate aligned with workspace guardrails.

## What these tests are protecting

- repository-facing configuration mutation remains typed and predictable
- the crate stays within its infrastructure boundary instead of growing product logic

## Verification

Useful commands from the repository root:

```sh
cargo test -p bijux-gnss-infra --test integration_overrides
cargo test -p bijux-gnss-infra --test integration_guardrails
```
