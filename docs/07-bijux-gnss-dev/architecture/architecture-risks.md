---
title: Architecture Risks
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Architecture Risks

## Main Risks

- the maintainer binary can become a catch-all bucket for unrelated automation
- governed-file validation can drift into undocumented hidden policy
- benchmark evidence can sprawl into unmanaged locations
- binary-only ownership can become hard to review if workflow families are not
  kept conceptually separate

## Mitigations

- add workflows only when the maintainer-owner reason is explicit
- keep governed inputs and outputs documented alongside command changes
- review new effects as boundary changes, not as incidental code cleanup
- split by durable workflow family if one-file ownership stops being readable

## First Proof Check

Inspect `crates/bijux-gnss-dev/docs/BOUNDARY.md`,
`crates/bijux-gnss-dev/docs/ARCHITECTURE.md`, and
`crates/bijux-gnss-dev/docs/OUTPUTS.md`. Then inspect
`crates/bijux-gnss-dev/src/main.rs` and
`crates/bijux-gnss-dev/tests/integration_guardrails.rs` to confirm the risk
register still matches the actual binary shape and effect model.
