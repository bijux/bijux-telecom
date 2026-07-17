---
title: Fixture And Simulation Care
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Fixture And Simulation Care

`bijux-gnss-receiver` relies on fixture-backed and synthetic proofs that carry
runtime meaning, not only test convenience.

## Care Rules

- treat truth-table, golden, and synthetic validation outputs as runtime proof
- when changing a fixture-backed behavior, explain whether the runtime or the
  expectation was wrong
- avoid silently broadening tolerances to make a runtime proof green
- keep synthetic helpers tied to the receiver boundary rather than letting them
  become a generic truth warehouse

## Why This Matters

This crate proves many claims through synthetic and integration evidence. Loose
fixture hygiene can make the suite look broad while weakening what it proves.
