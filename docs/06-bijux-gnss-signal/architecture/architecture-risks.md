---
title: Architecture Risks
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Architecture Risks

## Main Risks

- the signal crate can become a dumping ground for receiver-adjacent logic
  because many features touch signal math
- code-family growth can encourage duplicate near-equivalent helpers rather
  than one canonical signal implementation
- broad re-exports from `api.rs` can blur the difference between public
  contract and internal implementation detail
- raw-IQ metadata and sample utilities can drift toward repository ingestion
  policy if ownership is not defended

## Mitigations

- keep runtime and persistence behavior out of the crate
- add new code families by explicit signal meaning, not generic expansion
- review `api.rs` changes as public-boundary changes
- use the refusal ledger in [This Package Does Not Own](../this-package-does-not-own.md)
  when pressure repeats

## First Proof Check

Inspect `crates/bijux-gnss-signal/docs/BOUNDARY.md`,
`crates/bijux-gnss-signal/docs/ARCHITECTURE.md`, and
`crates/bijux-gnss-signal/docs/PUBLIC_API.md`. Then inspect
`crates/bijux-gnss-signal/src/api.rs`,
`crates/bijux-gnss-signal/src/dsp/mod.rs`, and
`crates/bijux-gnss-signal/tests/integration_guardrails.rs` to confirm the
architectural risk register still matches the enforced crate shape.
