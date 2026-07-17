---
title: Release and Versioning
audience: mixed
type: explanation
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Release and Versioning

Versioning pressure is stronger in `bijux-gnss-core` because downstream crates
inherit its record meanings.

## Versioning Rules

- prefer additive evolution when possible
- use explicit artifact payload version boundaries instead of silent rewrites
- treat public export changes as compatibility events, not as mere cleanup
- keep contract docs aligned with any release-significant shift in meaning

## Release Smell

If a release note would need to say "internal cleanup only" while `api.rs`,
artifact payloads, or core invariants changed, the change is being described
dishonestly.
