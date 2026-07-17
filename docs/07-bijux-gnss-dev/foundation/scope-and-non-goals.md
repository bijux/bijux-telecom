---
title: Scope And Non-Goals
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Scope And Non-Goals

## Scope

`bijux-gnss-dev` owns:

- maintainer commands that validate reviewed repository-governance files
- typed derivation of audit-ignore arguments from the reviewed allowlist
- benchmark comparison workflows that emit repository-scoped evidence
- guardrail tests that keep the maintainer crate narrow and honest

## Non-Goals

`bijux-gnss-dev` does not own:

- operator-facing product commands
- GNSS signal processing, navigation estimation, or receiver orchestration
- reusable library surfaces for product crates
- ad hoc shell conveniences with no stable repository-owner reason to exist

## Practical Rule

If a capability exists mainly because an operator needs it, a product crate
should own it. If it exists mainly because maintainers need explicit repository
governance, `bijux-gnss-dev` is the right candidate.

## First Proof Check

Read `crates/bijux-gnss-dev/docs/BOUNDARY.md`,
`crates/bijux-gnss-dev/docs/CONTRACTS.md`,
`crates/bijux-gnss-dev/docs/WORKFLOWS.md`, and
`crates/bijux-gnss-dev/docs/OUTPUTS.md`. Then inspect
`crates/bijux-gnss-dev/src/main.rs` to confirm the crate still stops at
maintainer governance instead of absorbing product commands or unguided
automation.
