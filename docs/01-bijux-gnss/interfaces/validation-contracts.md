---
title: Validation Contracts
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# Validation Contracts

Validation contracts define what the command crate owns when presenting
validation workflows to operators.

## Owned Validation Surfaces

- validation command families and argument shape
- command-boundary composition over lower-owner validation capabilities
- validation-facing output rendering and evidence-bundle publication

## Boundary Rule

The command crate owns the operator-facing validation workflow. The lower-level
meaning of repository artifacts, receiver validation, and navigation science
still belongs to their owning crates.

## Closest Proof

- `crates/bijux-gnss/src/cli/commands/validate/`
- `crates/bijux-gnss/docs/VALIDATION.md`

## Protecting Proof

Inspect `crates/bijux-gnss/src/cli/commands/validate/`,
`crates/bijux-gnss/docs/VALIDATION.md`,
`crates/bijux-gnss/tests/integration_validate_config.rs`, and
`crates/bijux-gnss/tests/integration_validate_synthetic_navigation.rs` to
confirm the operator-facing validation workflow still assembles lower-owner
proof honestly.
