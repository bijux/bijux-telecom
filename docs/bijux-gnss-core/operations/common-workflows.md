---
title: Common Workflows
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Common Workflows

Most core changes fall into one of four workflow families.

## Contract Clarification

You are documenting or tightening the meaning of an existing public record or
invariant. Update the handbook page, the crate-local contract doc, and the test
that proves the invariant together.

## Public Surface Change

You are adding or removing an export from `api.rs`. Treat this as a
cross-crate contract review first. Confirm the shared need, update public API
docs, and run the guardrail tests.

## Artifact Validation Change

You are changing payload validation or serialized meaning. Update
serialization-focused docs, validation tests, and any checked-in fixtures in
the same change set.

## Time Or Unit Correction

You are changing foundational physics or timekeeping behavior. Treat this as a
high-amplification change and keep property tests or regression fixtures in
scope from the start.
