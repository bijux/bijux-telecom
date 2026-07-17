---
title: Command Entry Contracts
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Command Entry Contracts

Each command has a narrow entry contract.

## Shared Entry Expectations

- every command may accept `--workspace-root` when the workflow needs repository
  root resolution
- maintainer diagnostics should explain which governed contract failed
- successful validation commands should report a simple pass condition

## Command-Specific Shape

- `audit-allowlist` validates exception records and expiry discipline
- `deny-policy-deviations` validates deviation ownership, review links, and
  expiry discipline
- `audit-ignore-args` derives exact `cargo audit --ignore` flags from reviewed
  inputs
- `bench-compare` accepts strictness and threshold controls for benchmark
  regression review
