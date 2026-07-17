---
title: Risk Register
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# Risk Register

This page records the main trust risks in `bijux-gnss-nav`.

## Risks

- a public export may become durable by accident because a downstream crate
  started using it
- reference-backed tests may stay green while a nearby scientific family
  quietly drifts if the wrong slice is exercised
- runtime-neutral helpers may invite runtime policy into the crate
- estimator evidence semantics may change more easily than users realize
- parser and precise-product families may absorb repository assumptions under
  validation pressure

## Mitigations

- review `api.rs` changes as public contract changes
- choose validation by scientific family, not by convenience
- keep refusal and evidence types documented and explicit
- maintain the refusal ledger in [This Package Does Not Own](../this-package-does-not-own.md)

## Protecting Proof

- `crates/bijux-gnss-nav/docs/PUBLIC_API.md`
- `crates/bijux-gnss-nav/docs/TESTS.md`
- `crates/bijux-gnss-nav/tests/integration_guardrails.rs`
