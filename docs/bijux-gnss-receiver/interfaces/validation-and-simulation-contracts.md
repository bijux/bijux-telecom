---
title: Validation And Simulation Contracts
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-18
---

# Validation And Simulation Contracts

Receiver validation and simulation prove receiver-boundary behavior. They
compare runtime outputs to reference truth, exercise synthetic receiver flows,
and package validation reports that explain runtime decisions. They do not own
repository artifact layout or the original truth sources.

## Proof Flow

```mermaid
flowchart LR
    truth["reference truth<br/>testkit or fixture"]
    runtime["receiver run<br/>acquisition tracking observations"]
    compare["reference validation<br/>alignment and budgets"]
    report["validation report<br/>runtime evidence"]
    consumers["tests, command reports,<br/>infra inspectors"]

    truth --> compare
    runtime --> compare --> report --> consumers
```

## Contract Families

| family | receiver-owned surface | reader promise |
| --- | --- | --- |
| reference alignment | reference-validation source | receiver outputs can be compared to reference epochs without repository policy |
| validation reports | validation-report source | runtime decisions, budgets, and integrity classifications stay typed |
| validation helpers | validation-helper source and report builders | receiver evidence can be summarized without hiding stage meaning |
| synthetic execution | synthetic simulation source | synthetic scenarios exercise acquisition, tracking, observations, and artifacts through the receiver boundary |
| covariance realism | covariance-realism source when navigation support is enabled | covariance claims remain tied to receiver-produced evidence |

## Boundary Decisions

- Truth fixtures may come from `bijux-gnss-testkit`, but receiver validation
  owns how runtime outputs are aligned and judged.
- Persisted artifact layout and repository inspection belong to infra after
  receiver artifacts exist.
- Signal and navigation crates own their scientific primitives; receiver
  simulation owns the runtime scenario that combines them.
- A validation report is a receiver contract when it explains runtime behavior,
  not merely because a test happens to use it.

## First Proof Check

Inspect the [receiver reference-validation guide](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-receiver/docs/REFERENCE_VALIDATION.md),
[simulation guide](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-receiver/docs/SIMULATION.md), and
[receiver test guide](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-receiver/docs/TESTS.md). Then
inspect reference-validation source, validation-report source, synthetic
simulation source, and focused integration tests for navigation validation runs
and synthetic receiver execution.
