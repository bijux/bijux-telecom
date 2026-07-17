# Public API

`bijux-gnss-policies` publishes one small Rust surface through `bijux_gnss_policies::api`.

## Exported items

- `check` runs guardrail validation for a crate root
- `GuardrailConfig` describes the rule configuration for a crate
- `GuardrailError` is the canonical failure type
- `Result` is the crate-local result alias

## Intent

The public surface is intentionally narrow. Downstream crates should be able to say “run the
guardrails for this crate” without importing internal rule modules or depending on internal file
layout.

## Extension rule

If a new export does not help another crate invoke or configure guardrails, it probably does not
belong in the public API.
