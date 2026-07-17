---
title: Reporting Contracts
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# Reporting Contracts

Reporting contracts define the operator-facing output shape owned by the
command crate.

## Owned Reporting Surfaces

- rendered success and failure output
- command-facing summaries over lower-owner artifacts
- diagnostics and validation publication at the command boundary

## Contract Rule

Reporting belongs here because operators consume it here. But reporting should
present lower-owner outcomes, not quietly redefine their scientific or
repository meaning.

## Closest Proof

- `crates/bijux-gnss/src/cli/report.rs`
- `crates/bijux-gnss/docs/REPORTING.md`

## Protecting Proof

Inspect `crates/bijux-gnss/src/cli/report.rs`,
`crates/bijux-gnss/docs/REPORTING.md`, and the reporting-facing workflow tests
under `crates/bijux-gnss/tests/` such as raw-IQ metrics, validation, and
synthetic export flows. Those proofs confirm that command-boundary reporting is
still presenting lower-owner results without redefining their meaning.
