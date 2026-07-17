---
title: Release And Versioning
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Release And Versioning

This crate is small in package version and large in repository impact.

## Release Discipline

- treat public export changes as cross-crate compatibility events
- treat raw-IQ metadata and validation-report changes as contract changes, not
  doc-only cleanup
- treat code-family and DSP behavior changes as repository-wide behavior moves
  unless proof shows they are isolated

## Versioning Reality

Even when the package version has not yet grown far, maintainers should act as
though downstream crates have already trusted these signal contracts. The safe
standard is behavioral discipline, not version-number optimism.

## First Proof Check

Inspect `crates/bijux-gnss-signal/docs/PUBLIC_API.md`,
`crates/bijux-gnss-signal/docs/CONTRACTS.md`, and
`crates/bijux-gnss-signal/docs/TESTS.md`. Then inspect
`crates/bijux-gnss-signal/src/api.rs`,
`crates/bijux-gnss-signal/tests/integration_signal_component_registry.rs`, and
`crates/bijux-gnss-signal/tests/integration_replica_continuity.rs` to confirm
that any claimed release safety still has public-contract and behavioral proof.
