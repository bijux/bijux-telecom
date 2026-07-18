---
title: Risk Register
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Risk Register

This page records the main trust risks in `bijux-gnss-receiver`.

## Risks

- a public export may become durable by accident because one downstream crate
  started using it
- stage integration tests may stay green while one local runtime family drifts
  if the wrong slice is exercised
- runtime artifacts may accumulate repository semantics because they are later
  persisted elsewhere
- receiver-owned validation or synthetic proof may quietly shift meaning
  without enough public scrutiny
- lower-owner science may be reimplemented locally under runtime pressure

## Mitigations

- review `api.rs` changes as public contract changes
- choose validation by runtime family, not by convenience
- keep artifact and validation meaning documented and explicit
- maintain placement decisions in the
  [receiver ownership boundaries](../ownership-boundaries.md)

## First Proof Check

Inspect `crates/bijux-gnss-receiver/docs/PUBLIC_API.md`,
`crates/bijux-gnss-receiver/docs/TESTS.md`,
`crates/bijux-gnss-receiver/tests/integration_basic.rs`, and the most relevant
artifact, validation, or stage-focused integration tests to confirm the main
runtime risk routes documented here still have active proof.
