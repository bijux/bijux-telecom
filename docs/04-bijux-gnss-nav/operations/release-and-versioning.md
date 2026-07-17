---
title: Release And Versioning
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# Release And Versioning

Navigation release notes should explain scientific contract movement, not only
file movement.

## Release Notes Should Mention

- which scientific family changed
- whether the change affects public API, only internal behavior, or both
- whether reference-backed expectations or tolerances moved
- whether downstream crates need to revisit assumptions

## Versioning Pressure Points

- new public exports in `api.rs`
- changed refusal or downgrade meaning
- changed interpretation of a supported external navigation product
- changed solver evidence that downstream crates present to users
