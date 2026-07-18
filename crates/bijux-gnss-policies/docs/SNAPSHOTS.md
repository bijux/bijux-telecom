# Snapshots

`bijux-gnss-policies` uses snapshots where policy configuration needs a durable
serialized shape. A snapshot is review evidence for policy behavior; it is not
noise to refresh without explanation.

## Snapshot Flow

```mermaid
flowchart LR
    config["GuardrailConfig"]
    snapshot["snapshot file"]
    test["snapshot test"]
    review["policy review"]

    config --> snapshot
    snapshot --> test
    test --> review
```

## Current Snapshot Surface

| snapshot | protects |
| --- | --- |
| `tests/snapshots/guardrail_default.json` | The default guardrail configuration shape, including rule-family names and serialized defaults. |

## What A Snapshot Change Means

A snapshot diff is meaningful when configuration is added, renamed, removed,
reordered in a semantically relevant way, or assigned a new default. It should
make policy drift visible before a downstream crate discovers the change through
a harder-to-debug guardrail failure.

## Change Discipline

- Explain the policy reason in the same change set.
- Keep [GUARDRAILS.md](GUARDRAILS.md) and [CONTRACTS.md](CONTRACTS.md) aligned
  when rule families or configuration meaning changes.
- Treat the snapshot as evidence of policy evolution, not generated clutter.
- Do not accept a snapshot diff only because a test told you to update it.

## Review Checks

- Can a reviewer tell which policy behavior changed?
- Does the snapshot still describe a stable serialized contract?
- Are downstream repository expectations affected by the new default?
