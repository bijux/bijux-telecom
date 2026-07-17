# Snapshots

`bijux-gnss-policies` uses snapshots where policy configuration needs a durable serialized shape.

## Current snapshot surface

- `tests/snapshots/guardrail_default.json`

## What the snapshot protects

The snapshot protects the default guardrail configuration shape. It makes policy drift visible when
configuration is added, renamed, removed, or reordered in a meaningful way.

## Change discipline

When the snapshot changes:

1. explain the policy reason in the same change set
2. keep [GUARDRAILS.md](GUARDRAILS.md) and [CONTRACTS.md](CONTRACTS.md) aligned when rule families
   or configuration meaning changes
3. treat the snapshot as evidence of policy evolution, not as noisy generated clutter
