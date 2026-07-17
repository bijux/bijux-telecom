---
title: Quality
audience: mixed
type: index
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Quality

Open this section when the question is whether `bijux-gnss-signal` is proving
its reusable signal claims honestly enough.

## Read These First

- open [Test Strategy](test-strategy.md) first when the question is how proof
  is distributed across this crate
- open [Invariants](invariants.md) when the question is what must remain true
  even as supported signals grow
- open [Change Validation](change-validation.md) when the question is the
  minimum honest proof for a change

## Pages In This Section

- [Test Strategy](test-strategy.md)
- [Invariants](invariants.md)
- [Change Validation](change-validation.md)
- [Review Checklist](review-checklist.md)
- [Definition Of Done](definition-of-done.md)
- [Known Limitations](known-limitations.md)
- [Risk Register](risk-register.md)

## First Proof Surfaces

- `crates/bijux-gnss-signal/tests/`
- `crates/bijux-gnss-signal/docs/TESTS.md`
- `crates/bijux-gnss-signal/docs/VALIDATION.md`

## Leave This Section When

- leave for [Operations](../operations/) when the quality bar is clear and the
  next question is execution sequence
- leave for [Foundation](../foundation/) when the doubt is still about whether
  the crate should own the behavior at all
