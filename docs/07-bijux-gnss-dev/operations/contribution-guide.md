---
title: Contribution Guide
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Contribution Guide

This page replaces the old root-level contributing note. It gives maintainers
the shortest honest sequence for changing repository-owned surfaces.

## Expected Change Sequence

1. confirm the owning crate and keep the change inside that boundary
2. preserve durable naming and remove thin or duplicate structure instead of
   extending it
3. run the narrowest honest verification for the affected package or
   maintainer workflow
4. update the owning handbook or crate docs when the public or reviewed meaning
   changed
5. commit one coherent intent at a time

## Minimum Gate Expectation

Changes that touch governed repository workflows should prove themselves with
the relevant `make` lane or a narrower crate-local command that directly backs
that lane. Do not rely on unrelated green tests as substitute proof.

## First Proof Check

- `crates/bijux-gnss-dev/docs/WORKFLOWS.md`
- `crates/bijux-gnss-dev/docs/TESTS.md`
- `docs/07-bijux-gnss-dev/operations/verification-commands.md`

Contributors should also inspect `crates/bijux-gnss-dev/docs/OUTPUTS.md` and
`crates/bijux-gnss-dev/docs/GOVERNANCE_FILES.md` before changing a command.
Those files make the read and write contract explicit enough to keep
maintainer-only tooling from drifting into generic repository automation.
