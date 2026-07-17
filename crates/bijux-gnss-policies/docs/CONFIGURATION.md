# Configuration

`bijux-gnss-policies` owns typed configuration for repository guardrails.

## Configuration surface

`GuardrailConfig` currently controls:

- file-size and source-depth limits
- directory fan-out limits
- public-item and `pub use` density limits
- panic/expect and staged-string restrictions
- purity-zone path and regex restrictions
- path allowlists for intentionally narrow exceptions

## Why this matters

Without typed configuration, repository policy drifts into scattered literals and special cases.
This crate keeps the policy knobs explicit so maintainers can see which rules are general and which
exceptions are deliberate.

## Boundary rule

This configuration is for repository structure and policy enforcement. It is not application
configuration for runtime behavior.
