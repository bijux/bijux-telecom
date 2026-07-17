---
title: Architecture
audience: mixed
type: index
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Architecture

Open this section when the question is structural: where contract families live
in code, how dependency direction is enforced, and how the core crate keeps a
foundational shape instead of becoming a behavior-heavy library.

This section will stay tightly tied to `src/api.rs`, `src/artifact/`,
`src/observation/`, `src/nav_solution.rs`, and the guardrail tests that stop
higher-level dependencies from leaking inward.

## First Proof Check

- `crates/bijux-gnss-core/src/api.rs`
- `crates/bijux-gnss-core/src/artifact/`
- `crates/bijux-gnss-core/src/observation/`
- `crates/bijux-gnss-core/docs/ARCHITECTURE.md`

## Leave This Section When

- leave for [Foundation](../foundation/) when the real disagreement is still
  about ownership rather than structure
- leave for [Interfaces](../interfaces/) when the structural question is now a
  public contract question
- leave for [Quality](../quality/) when the structure is clear and the next
  question is whether the proofs are strong enough
