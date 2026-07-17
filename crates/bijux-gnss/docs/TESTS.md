# Tests

## Entry points

- acquisition and synthetic-signal workflow integration tests
- raw-IQ metadata and front-end metrics integration tests
- navigation decode and RINEX workflow tests
- capture, config, synthetic IQ, synthetic navigation, and bias-validation tests
- `tests/integration_guardrails.rs` for workspace guardrail coverage

## What these tests are protecting

- command handlers remain wired to the correct lower-level surfaces
- operator-facing workflows continue to produce coherent validation and reporting behavior
- the CLI boundary does not silently drift when lower-level crates evolve

## Verification

Useful commands from the repository root:

```sh
cargo test -p bijux-gnss --test integration_validate_config
cargo test -p bijux-gnss --test integration_nav_decode
cargo test -p bijux-gnss --test integration_validate_synthetic_navigation
```
