# Tests

## Entry points

- `tests/scientific_independence.rs` checks that key truth helpers do not call back into forbidden
  `nav` helper paths.
- `tests/integration_guardrails.rs` keeps the crate aligned with workspace guardrails.

## What these tests are protecting

- the testkit remains a truth source rather than a reflection of production helper internals
- the crate keeps its shared-fixture role without turning into an unbounded misc bucket
- fixture loading and truth-model boundaries remain explicit enough for other crates to trust them

## Verification

Useful commands from the repository root:

```sh
cargo test -p bijux-gnss-testkit --test scientific_independence
cargo test -p bijux-gnss-testkit --test integration_guardrails
```
