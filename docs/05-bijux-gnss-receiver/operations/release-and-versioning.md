---
title: Release And Versioning
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Release And Versioning

Receiver release notes should explain runtime contract movement, not only file
movement.

## Release Notes Should Mention

- which runtime family changed
- whether the change affects public API, only internal behavior, or both
- whether artifact, validation, or synthetic proof expectations moved
- whether downstream crates need to revisit assumptions

## Versioning Pressure Points

- new public exports in `api.rs`
- changed runtime artifact meaning
- changed port or runtime-sink behavior
- changed validation-report or synthetic receiver evidence that downstream
  crates present to users

## First Proof Check

Inspect `crates/bijux-gnss-receiver/docs/PUBLIC_API.md`,
`crates/bijux-gnss-receiver/docs/RUNTIME.md`,
`crates/bijux-gnss-receiver/docs/ARTIFACTS.md`,
`crates/bijux-gnss-receiver/docs/REFERENCE_VALIDATION.md`, and
`crates/bijux-gnss-receiver/src/api.rs`. Then inspect the most relevant stage,
artifact, or validation tests to confirm any claimed release safety still
matches public runtime meaning.
