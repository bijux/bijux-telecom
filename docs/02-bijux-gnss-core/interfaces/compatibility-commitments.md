---
title: Compatibility Commitments
audience: mixed
type: explanation
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Compatibility Commitments

Compatibility in `bijux-gnss-core` is more about semantic stability than about
never changing a type.

## Commitments

- public exports should remain curated through `api.rs`
- additive evolution is preferred over silent semantic rewrites
- serialized artifact meaning should change through explicit version boundaries
- invariant changes should be documented and proven in the same change set

## Non-Commitments

- private module layout is not a public promise
- one owner’s local convenience helper is not owed cross-crate stability
- repository file layout is not part of the core crate compatibility contract

## Protecting Proof

- `crates/bijux-gnss-core/docs/CHANGE_RULES.md`
- `crates/bijux-gnss-core/docs/SERIALIZATION.md`
- `crates/bijux-gnss-core/tests/public_api_guardrail.rs`
