---
title: Definition Of Done
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# Definition Of Done

A navigation change is done when the scientific claim is explicit, the owning
model or estimator family is named, and the selected proof defends the same
claim. Parser success, correction success, and solution success are related but
not interchangeable.

## Completion Gate

| changed surface | done means | proof to start from |
| --- | --- | --- |
| format or product parsing | decoded records and rejection reasons match the product contract | `crates/bijux-gnss-nav/docs/FORMATS.md` plus parser/product tests |
| orbit, clock, time, or model behavior | physical assumptions and reference accuracy remain named and bounded | `ORBITS.md`, `TIME.md`, `MODELS.md`, and focused model tests |
| corrections and combinations | correction applicability remains reusable across solvers that consume it | `CORRECTIONS.md` plus the relevant ionosphere, troposphere, bias, or combination test |
| SPP, PPP, RTK, or integrity | estimator claims, downgrade behavior, residuals, and refusal conditions remain inspectable | `ESTIMATION.md` plus focused solution tests |
| public API | exported items are durable navigation contracts, not solver-local shortcuts | `PUBLIC_API.md` and guardrail proof |

## Reader Questions Before Commit

- What scientific family owns the change?
- Which constellation, product type, model, estimator, or fixture scope is
  actually covered?
- Does any shared record meaning need to move through `bijux-gnss-core`?
- Does receiver, infra, or command code only consume the nav result, or did it
  force nav to absorb another owner?
- What exact proof would fail if the scientific claim regressed?

## Proof Route

1. Read `crates/bijux-gnss-nav/docs/BOUNDARY.md`.
2. Read the family page: `FORMATS.md`, `ORBITS.md`, `TIME.md`, `MODELS.md`,
   `CORRECTIONS.md`, or `ESTIMATION.md`.
3. Choose the focused test family from `crates/bijux-gnss-nav/docs/TESTS.md`.
4. Update this handbook when public scientific meaning or reader routing moves.

Do not call a navigation change done because one broad solution test passed.
The proof must match the exact parser, model, correction, estimator, or public
API claim in the change.
