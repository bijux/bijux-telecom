---
title: Quality
audience: mixed
type: index
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Quality

Open this section when the question is whether `bijux-gnss-receiver` is
proving its runtime claims honestly enough: invariants, test strategy,
limitations, change validation, and risk.

## Trust Model

```mermaid
flowchart LR
    tests["test strategy"]
    invariants["invariants"]
    validation["change validation"]
    limits["limitations and risks"]
    trust["trust decision"]

    tests --> invariants --> validation --> limits --> trust
```

## Read These First

- open [Test Strategy](test-strategy.md) first when you need the broad proof
  shape
- open [Invariants](invariants.md) when the question is what callers may safely
  assume about receiver behavior
- open [Change Validation](change-validation.md) when you need the minimum
  proof for a safe receiver change

## Pages In This Section

- [Test Strategy](test-strategy.md)
- [Invariants](invariants.md)
- [Change Validation](change-validation.md)
- [Determinism And Purity](determinism-and-purity.md)
- [Validation Budgets](validation-budgets.md)
- [Review Checklist](review-checklist.md)
- [Definition Of Done](definition-of-done.md)
- [Known Limitations](known-limitations.md)
- [Risk Register](risk-register.md)

## First Proof Check

- `crates/bijux-gnss-receiver/tests/`
- `crates/bijux-gnss-receiver/docs/TESTS.md`
- crate-local docs for runtime, pipeline, ports, artifacts, validation, and
  simulation

## Leave This Section When

- leave for [Foundation](../foundation/) when the doubt is still about what the
  crate should own
- leave for [Interfaces](../interfaces/) when the real question is what public
  runtime promise exists, not how well it is defended
