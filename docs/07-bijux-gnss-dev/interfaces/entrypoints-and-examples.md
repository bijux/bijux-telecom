---
title: Entrypoints And Examples
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Entrypoints And Examples

Use these entrypoints when choosing the right maintainer command.

## Common Starting Points

- run `bijux-gnss-dev audit-allowlist` when reviewed audit exceptions change
- run `bijux-gnss-dev deny-policy-deviations` when deviation governance changes
- run `bijux-gnss-dev audit-ignore-args` when automation needs exact ignore
  flags from the reviewed allowlist
- run `bijux-gnss-dev bench-compare --strict` when benchmark regressions should
  fail the workflow

## Example Sequence

If a maintainer updates a reviewed security exception, the durable sequence is:

1. edit `audit-allowlist.toml`
2. run `bijux-gnss-dev audit-allowlist`
3. run `bijux-gnss-dev audit-ignore-args` if automation consumes the derived
   flags
