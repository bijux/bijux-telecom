---
title: Integration Seams
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Integration Seams

The binary stays honest because its workflows connect through explicit reviewed
inputs and evidence outputs instead of through hidden product coupling.

## Main Seams

- command parser to workflow functions:
  `Commands` selects one maintainer-owned flow explicitly
- reviewed files to validators:
  audit and deviation commands consume named repository files and validate their
  documented contract
- benchmark runner to evidence outputs:
  benchmark commands emit raw and normalized evidence into governed locations
- tests to repository structure:
  the nextest-roster test binds maintainer policy to actual repository test
  names without pretending the roster is a runtime command input

## Integration Rule

Each seam should remain inspectable from the binary entry point. If a reviewer
cannot tell what a command reads, writes, or invokes by reading the command
path, the maintainer boundary is becoming opaque.

## First Proof Check

Inspect `crates/bijux-gnss-dev/docs/WORKFLOWS.md`,
`crates/bijux-gnss-dev/docs/OUTPUTS.md`,
`crates/bijux-gnss-dev/docs/TESTS.md`, and
`crates/bijux-gnss-dev/src/main.rs`. Then inspect both integration tests to
confirm the seams remain explicit command-to-governed-file or
command-to-evidence routes rather than hidden cross-repository coupling.
