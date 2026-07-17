---
title: Command Surface
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Command Surface

The durable public surface of `bijux-gnss-dev` is its binary command set.

## Owned Commands

- `audit-allowlist`
- `deny-policy-deviations`
- `audit-ignore-args`
- `bench-compare`

## Public Promise

Each command exists because it owns a reviewed maintainer workflow, not because
the repository happens to need another root-level utility. The command list is
small on purpose.
