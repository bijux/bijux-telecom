---
title: bijux-gnss-core
audience: mixed
type: index
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# bijux-gnss-core

`bijux-gnss-core` owns the shared scientific and operational contracts that the
rest of `bijux-gnss` depends on. This is where identifiers, units, time
systems, observation records, diagnostics, and artifact envelopes become
durable enough for higher-level crates to build on without reinterpreting the
same meaning in seven places.

If another crate is the action layer, `bijux-gnss-core` is the meaning layer.
It should be boring in the best possible way: precise, typed, reusable, and
hard to misread.

```mermaid
flowchart LR
    core["bijux-gnss-core<br/>shared GNSS meaning"]
    signal["bijux-gnss-signal<br/>signal primitives"]
    nav["bijux-gnss-nav<br/>navigation science"]
    receiver["bijux-gnss-receiver<br/>runtime execution"]
    infra["bijux-gnss-infra<br/>persisted evidence"]
    gnss["bijux-gnss<br/>public commands"]

    core --> signal
    core --> nav
    core --> receiver
    core --> infra
    core --> gnss
```

## Read These First

- open [Foundation](foundation/index.md) when the question is why the crate exists,
  what it owns, and where it should refuse more work
- open [Interfaces](interfaces/index.md) when the dispute is already about public
  imports, artifact envelopes, observation records, or configuration-facing
  contracts
- open [Architecture](architecture/index.md) when the question is structural: which
  module owns which contract family and how the crate stays dependency-light
- open [Quality](quality/index.md) when the boundary is clear and the question becomes
  whether the proofs are strong enough

## Why This Package Exists

- cross-package GNSS meaning must be defined once before signal, navigation,
  receiver, and infrastructure crates build on it
- versioned artifacts and observation records need one canonical contract owner
- shared diagnostics, IDs, time conversions, and units should not drift by
  crate

## What It Owns

- canonical identifiers for constellations, satellites, signals, and
  supporting matrices
- units, geometry, and time-system conversion contracts
- acquisition, tracking, observation, differencing, and navigation-solution
  records
- shared diagnostics, config validation helpers, and crate-foundational error
  categories
- versioned artifact envelopes and payload validation rules

## What It Refuses

- raw-IQ ingestion and sample-source behavior owned by `bijux-gnss-signal`
- dataset registry, run layout, or experiment persistence owned by
  `bijux-gnss-infra`
- navigation estimation strategies owned by `bijux-gnss-nav`
- receiver scheduling, stage orchestration, and runtime policy owned by
  `bijux-gnss-receiver`
- operator-facing commands owned by `bijux-gnss`

## Strongest Proof Surfaces

- crate README:
  [Core crate README](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-core/README.md)
- public contract docs:
  [Core public API](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-core/docs/PUBLIC_API.md),
  [Core contracts](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-core/docs/CONTRACTS.md)
- invariant and serialization docs:
  [Core invariants](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-core/docs/INVARIANTS.md),
  [Serialization contracts](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-core/docs/SERIALIZATION.md)
- source roots:
  [curated core API source](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-core/src/api.rs),
  [artifact contract source](https://github.com/bijux/bijux-gnss/tree/main/crates/bijux-gnss-core/src/artifact),
  [observation contract source](https://github.com/bijux/bijux-gnss/tree/main/crates/bijux-gnss-core/src/observation)
- proof tests:
  [core contract tests](https://github.com/bijux/bijux-gnss/tree/main/crates/bijux-gnss-core/tests)

## Support Crates That Matter Here

- `bijux-gnss-policies` helps defend core dependency direction and public API
  guardrails; inspect it when a contract-layer change also changes repository
  structure expectations.
- `bijux-gnss-testkit` is not a production owner here, but it is an important
  stress consumer of shared GNSS contracts; inspect it when a core change is
  justified by test truth, reference models, or cross-crate fixture reuse.

## Sections In This Handbook

- [Foundation](foundation/index.md) for role, scope, ownership, repository fit, and
  contract-language discipline
- [Architecture](architecture/index.md) for module layout, dependency direction,
  extensibility, and code navigation
- [Interfaces](interfaces/index.md) for public imports, artifact envelopes,
  observation contracts, navigation-solution records, and examples
- [Operations](operations/index.md) for safe change sequence, local verification, and
  maintenance workflows around the crate
- [Quality](quality/index.md) for invariants, trust boundaries, validation commands,
  and known risks
- [Shared contract ownership boundaries](ownership-boundaries.md) for deciding
  whether a type or invariant is truly cross-package

## Start Here When

- the question is about the shape or semantics of data shared across crates
- the reader needs to know where IDs, units, times, or artifact records become
  canonical
- a higher-level crate seems to be inventing meaning it should have inherited
- the issue is whether an artifact or observation record is valid independent
  of runtime or CLI policy

## Reader Questions This Package Can Answer

- what a tracked observation, navigation solution, or artifact envelope means
- which time and coordinate conventions are repository law rather than local
  convenience
- where diagnostic codes and shared support matrices become canonical
- whether a change belongs in a downstream crate or in the cross-package
  contract layer first

## Leave This Handbook When

- the question becomes about persisted dataset or run layout behavior:
  [Infra handbook](../bijux-gnss-infra/index.md)
- the question becomes about signal codes, raw-IQ contracts, or DSP:
  [Signal handbook](../bijux-gnss-signal/index.md)
- the question becomes about navigation estimation or orbit products:
  [Navigation handbook](../bijux-gnss-nav/index.md)
- the question becomes about runtime composition or stage execution:
  [Receiver handbook](../bijux-gnss-receiver/index.md)

## First Proof Check

- [curated core API source](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-core/src/api.rs)
- [identity contracts](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-core/src/ids.rs)
- [time contracts](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-core/src/time.rs)
- [unit contracts](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-core/src/units.rs)
- [observation contract source](https://github.com/bijux/bijux-gnss/tree/main/crates/bijux-gnss-core/src/observation)
- [artifact contract source](https://github.com/bijux/bijux-gnss/tree/main/crates/bijux-gnss-core/src/artifact)
- [core contract map](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-core/docs/CONTRACT_MAP.md)

## Design Pressure

If `bijux-gnss-core` starts carrying runtime policy, repository persistence, or
crate-local convenience helpers that one downstream owner could define for
itself, the shared contract layer becomes weaker rather than stronger.
