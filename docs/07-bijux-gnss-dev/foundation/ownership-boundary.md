---
title: Ownership Boundary
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Ownership Boundary

The most important discipline in `bijux-gnss-dev` is keeping repository-health
workflows separate from product behavior and from generic scripting sprawl.

## This Crate Owns

- maintainer-only command parsing and execution
- reviewed governance-file validation
- exact derivation of audit-ignore arguments from reviewed inputs
- repository-scoped benchmark comparison and evidence emission

## Neighbor Crates Own The Rest

- `bijux-gnss` owns public command workflows and operator-facing reports
- `bijux-gnss-receiver` owns runtime execution and receiver artifacts
- product crates own GNSS science and reusable product APIs
- infrastructure and repository policy neighbors own long-lived repository data
  that is not maintainer evidence

## Boundary Test

When a proposed command says "it is convenient to have this near other dev
commands," that is not enough. The real question is whether the workflow has a
durable maintainer-owner reason to exist as reviewed repository tooling.

## First Proof Check

Read `crates/bijux-gnss-dev/docs/BOUNDARY.md`,
`crates/bijux-gnss-dev/docs/CONTRACTS.md`, and
`crates/bijux-gnss-dev/docs/WORKFLOWS.md` first. Then inspect
`crates/bijux-gnss-dev/src/main.rs`,
`crates/bijux-gnss-dev/tests/integration_guardrails.rs`, and
`crates/bijux-gnss-dev/tests/integration_nextest_suite_selection.rs` to
confirm the binary still owns maintainer workflows rather than product
behavior or generic automation sprawl.
