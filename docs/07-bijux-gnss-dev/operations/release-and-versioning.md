---
title: Release And Versioning
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Release And Versioning

This crate is unpublished, but its workflow meaning is still a compatibility
surface for the repository.

## Release Discipline

- treat command meaning changes as maintainer workflow changes
- treat governed input and output changes as repository contract changes
- treat benchmark-evidence behavior changes as review-relevant even when no
  product crate code moved

## Versioning Reality

Even without a published library surface, maintainers already depend on the
binary's documented behavior. The right standard is workflow stability, not
version-number comfort.

## First Proof Check

Inspect `crates/bijux-gnss-dev/docs/COMMANDS.md`,
`crates/bijux-gnss-dev/docs/WORKFLOWS.md`,
`crates/bijux-gnss-dev/docs/OUTPUTS.md`, and
`crates/bijux-gnss-dev/src/main.rs`. Then inspect the integration tests to
confirm that any claimed release safety still matches the maintained command
surface and evidence contract.
