---
title: Risk Register
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# Risk Register

This page records the main trust risks in `bijux-gnss`.

## Risks

- a command or flag may become durable by accident without enough review
- workflow integration tests may stay green while the command boundary quietly
  drifts if the wrong slice is exercised
- the Rust facade may accumulate convenience exports that blur lower ownership
- reporting may start encoding lower-owner policy rather than presenting it

## Mitigations

- review command-shape changes as public contract changes
- choose validation by command family, not by convenience
- keep facade scope narrow and documented
- maintain the refusal ledger in [This Package Does Not Own](../this-package-does-not-own.md)

## Protecting Proof

- `crates/bijux-gnss/docs/COMMANDS.md`
- `crates/bijux-gnss/docs/FACADE.md`
- `crates/bijux-gnss/tests/integration_guardrails.rs`
