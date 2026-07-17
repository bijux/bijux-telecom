---
title: Error Model
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# Error Model

The command crate has more than one honest failure mode, and the architecture
should keep them distinct.

## Main Failure Families

- invalid command shape or argument usage
- environment or input-loading failure during command setup
- lower-level runtime, repository, or scientific failures surfaced through the
  command boundary
- operator-facing validation or report failures that indicate a bad workflow
  outcome rather than a malformed command

## Why Distinction Matters

A bad flag is not the same as a receiver runtime failure, and neither is the
same as a validation report that shows the workflow outcome is poor. If those
cases collapse into one generic command error story, operator trust gets weaker.

## Closest Proof

- `crates/bijux-gnss/src/cli/command_line.rs`
- `crates/bijux-gnss/src/cli/command_runtime/`
- `crates/bijux-gnss/src/cli/report.rs`
