---
title: Code Navigation
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Code Navigation

When you need to inspect the crate quickly, use this route order.

## Fast Reading Path

1. start at the [crate root](../../../crates/bijux-gnss-core/src/lib.rs) to
   confirm the crate exports one deliberate public facade
2. read the [public API facade](../../../crates/bijux-gnss-core/src/api.rs) to
   see what the crate claims publicly
3. jump to the owning family:
   artifacts, observations, navigation solutions, identifiers, time, units,
   geodesy, diagnostics, support inventory, or configuration
4. read the corresponding crate-local guides, especially the
   [public API guide](../../../crates/bijux-gnss-core/docs/PUBLIC_API.md),
   [contract guide](../../../crates/bijux-gnss-core/docs/CONTRACTS.md),
   [contract map](../../../crates/bijux-gnss-core/docs/CONTRACT_MAP.md),
   [invariants guide](../../../crates/bijux-gnss-core/docs/INVARIANTS.md), and
   [serialization guide](../../../crates/bijux-gnss-core/docs/SERIALIZATION.md)
5. confirm with the narrow integration or property test that protects the
   contract

## Review Shortcut

If a change touches the public facade, artifact records, observation records,
navigation-solution records, or time model, treat it as a cross-crate contract
review first and an implementation review second.

## First Proof Check

- the [crate root](../../../crates/bijux-gnss-core/src/lib.rs)
- the [public API facade](../../../crates/bijux-gnss-core/src/api.rs)
- the [navigation artifact validation](../../../crates/bijux-gnss-core/tests/nav_artifact_validation.rs)
- the [tracking artifact validation](../../../crates/bijux-gnss-core/tests/tracking_artifact_validation.rs)
- the [timekeeping property tests](../../../crates/bijux-gnss-core/tests/prop_timekeeping.rs)
- the [public API guardrail](../../../crates/bijux-gnss-core/tests/public_api_guardrail.rs)
