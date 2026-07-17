# Change Rules

`bijux-gnss-policies` changes repository behavior even when no product code moves. That makes its
documentation and change discipline unusually important.

## Required discipline

1. If a new guardrail family is added, update [GUARDRAILS.md](GUARDRAILS.md).
2. If configuration meaning changes, update [CONFIGURATION.md](CONFIGURATION.md) and any affected
   snapshot references.
3. If a reporting surface changes, update [REPORTING.md](REPORTING.md).
4. If a snapshot changes, explain the policy reason in the same change set.

## Smells that usually mean "wrong crate"

- code that mutates product state
- code that belongs to runtime behavior rather than repository policy
- helpers added only because a test needed a convenient utility

This crate should remain policy-specific and review-friendly.
