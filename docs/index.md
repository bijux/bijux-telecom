---
title: GNSS Workspace Handbook
audience: mixed
type: index
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-18
---

# GNSS Workspace Handbook

Use this handbook to find the package that owns a GNSS behavior and the
evidence needed to trust it. The workspace separates operator commands, shared
meaning, persisted evidence, navigation science, receiver execution, reusable
signal processing, and repository maintenance so that each claim has a clear
owner.

This page is a map, not an API reference or a scientific authority. Once you
identify the owner, continue in that package's handbook, crate documentation,
source, and tests.

<!-- bijux-gnss-badges:generated:start -->
[![License: Apache-2.0](https://img.shields.io/badge/license-Apache--2.0-0F766E)](https://github.com/bijux/bijux-gnss/blob/main/LICENSE)
[![CI](https://github.com/bijux/bijux-gnss/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/bijux/bijux-gnss/actions/workflows/ci.yml?query=branch%3Amain)
[![Docs](https://github.com/bijux/bijux-gnss/workflows/deploy-docs/badge.svg)](https://github.com/bijux/bijux-gnss/actions/workflows/deploy-docs.yml)
[![Release](https://img.shields.io/github/v/release/bijux/bijux-gnss?display_name=tag&label=release)](https://github.com/bijux/bijux-gnss/releases)
[![GHCR targets](https://img.shields.io/badge/ghcr%20targets-6%20packages-181717?logo=github)](https://github.com/bijux?tab=packages&repo_name=bijux-gnss)
[![Public crates](https://img.shields.io/badge/public%20crates-6-2563EB)](https://github.com/bijux/bijux-gnss/tree/main/crates)

[![bijux-gnss](https://img.shields.io/crates/v/bijux-gnss?label=bijux--gnss&logo=rust)](https://crates.io/crates/bijux-gnss)
[![bijux-gnss-core](https://img.shields.io/crates/v/bijux-gnss-core?label=core&logo=rust)](https://crates.io/crates/bijux-gnss-core)
[![bijux-gnss-infra](https://img.shields.io/crates/v/bijux-gnss-infra?label=infra&logo=rust)](https://crates.io/crates/bijux-gnss-infra)
[![bijux-gnss-nav](https://img.shields.io/crates/v/bijux-gnss-nav?label=nav&logo=rust)](https://crates.io/crates/bijux-gnss-nav)
[![bijux-gnss-receiver](https://img.shields.io/crates/v/bijux-gnss-receiver?label=receiver&logo=rust)](https://crates.io/crates/bijux-gnss-receiver)
[![bijux-gnss-signal](https://img.shields.io/crates/v/bijux-gnss-signal?label=signal&logo=rust)](https://crates.io/crates/bijux-gnss-signal)

[![ghcr-bijux--gnss](https://img.shields.io/badge/ghcr-bijux--gnss-181717?logo=github)](https://github.com/bijux/bijux-gnss/pkgs/container/bijux-gnss%2Fbijux-gnss)
[![ghcr-core](https://img.shields.io/badge/ghcr-core-181717?logo=github)](https://github.com/bijux/bijux-gnss/pkgs/container/bijux-gnss%2Fbijux-gnss-core)
[![ghcr-infra](https://img.shields.io/badge/ghcr-infra-181717?logo=github)](https://github.com/bijux/bijux-gnss/pkgs/container/bijux-gnss%2Fbijux-gnss-infra)
[![ghcr-nav](https://img.shields.io/badge/ghcr-nav-181717?logo=github)](https://github.com/bijux/bijux-gnss/pkgs/container/bijux-gnss%2Fbijux-gnss-nav)
[![ghcr-receiver](https://img.shields.io/badge/ghcr-receiver-181717?logo=github)](https://github.com/bijux/bijux-gnss/pkgs/container/bijux-gnss%2Fbijux-gnss-receiver)
[![ghcr-signal](https://img.shields.io/badge/ghcr-signal-181717?logo=github)](https://github.com/bijux/bijux-gnss/pkgs/container/bijux-gnss%2Fbijux-gnss-signal)

[![Repository docs](https://img.shields.io/badge/docs-repository-2563EB?logo=materialformkdocs&logoColor=white)](https://github.com/bijux/bijux-gnss/tree/main/docs)
[![bijux-gnss rust-docs](https://img.shields.io/badge/rust--docs-bijux--gnss-DEA584?logo=rust&logoColor=white)](https://docs.rs/bijux-gnss/latest/bijux_gnss/)
<!-- bijux-gnss-badges:generated:end -->

## Find The Owner

| question | owning handbook | strongest starting evidence |
| --- | --- | --- |
| What does an operator command accept, execute, or report? | [Command workflows](bijux-gnss/index.md) | [command contracts](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss/docs/COMMANDS.md) and [workflow implementation](https://github.com/bijux/bijux-gnss/tree/main/crates/bijux-gnss/src/cli) |
| What do shared identities, units, times, diagnostics, and artifact envelopes mean? | [Shared GNSS contracts](bijux-gnss-core/index.md) | [core contracts](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-core/docs/CONTRACTS.md) and [public facade](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-core/src/api.rs) |
| How are datasets, run identities, overrides, and persisted evidence handled? | [Repository infrastructure](bijux-gnss-infra/index.md) | [infrastructure contracts](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-infra/docs/CONTRACTS.md) and [run layout](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-infra/docs/RUN_LAYOUT.md) |
| Which parser, orbit model, correction, or estimator owns a navigation result? | [Navigation science](bijux-gnss-nav/index.md) | [navigation contracts](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-nav/docs/CONTRACTS.md) and [navigation test evidence](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-nav/docs/TESTS.md) |
| How are acquisition, tracking, observations, and runtime validation staged? | [Receiver execution](bijux-gnss-receiver/index.md) | [pipeline contract](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-receiver/docs/PIPELINE.md) and [receiver tests](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-receiver/docs/TESTS.md) |
| Which signal identities, code families, sample contracts, and DSP primitives are reusable? | [Signal processing](bijux-gnss-signal/index.md) | [signal architecture](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-signal/docs/ARCHITECTURE.md) and [signal tests](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-signal/docs/TESTS.md) |
| Which typed command validates repository governance or benchmark evidence? | [Maintainer tooling](bijux-gnss-dev/index.md) | [maintainer commands](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-dev/docs/COMMANDS.md) and [workflow behavior](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-dev/docs/WORKFLOWS.md) |
| What changed across the workspace? | [Workspace changelog](https://github.com/bijux/bijux-gnss/blob/main/CHANGELOG.md) | the affected package changelog and commit history |

```mermaid
flowchart LR
    question["reader question"]
    owner{"who makes the<br/>authoritative decision?"}
    handbook["package handbook"]
    contract["crate contract<br/>or public API"]
    implementation["implementation"]
    evidence["tests, reference data,<br/>and run artifacts"]

    question --> owner --> handbook
    handbook --> contract
    contract --> implementation
    implementation --> evidence
```

## Follow A Receiver Result

An operator-visible result often crosses several packages without transferring
ownership:

```mermaid
flowchart LR
    command["command selects<br/>a workflow"]
    infra["infrastructure resolves<br/>inputs and persistence"]
    receiver["receiver stages<br/>runtime execution"]
    signal["signal supplies<br/>physical primitives"]
    nav["navigation estimates<br/>state and corrections"]
    core["core carries shared<br/>typed records"]
    report["command renders<br/>the result"]

    command --> infra --> receiver
    signal --> receiver
    receiver --> nav
    core --> infra
    core --> receiver
    core --> signal
    core --> nav
    nav --> report
    receiver --> report
```

Trace a failure to the package that made the disputed decision:

- command parsing and report wording remain command concerns;
- dataset discovery, run identity, and persistence remain infrastructure
  concerns;
- acquisition, tracking, channel state, and observation production remain
  receiver concerns;
- code generation, carrier relationships, sample meaning, and reusable DSP
  remain signal concerns;
- orbit interpretation, corrections, estimators, and solution acceptance
  remain navigation concerns;
- shared records and units remain core concerns even when another package
  creates their values.

## Choose Evidence That Matches The Claim

| claim | evidence to prefer | evidence that is not enough |
| --- | --- | --- |
| a public command behaves as documented | command integration test plus rendered output | a helper unit test |
| a shared record preserves meaning | core contract, serialization proof, and consumer test | successful construction alone |
| a persisted run can be trusted | manifest, provenance, validation status, and referenced files | directory presence |
| an algorithm is scientifically correct | independent reference, truth budget, and failure diagnostics | a self-generated fixture |
| receiver state is operationally valid | per-stage evidence, lifecycle status, and bounded errors | a final position alone |
| signal behavior is physically coherent | reference vectors, properties, and continuity checks | a plausible waveform plot |
| a maintenance gate enforced policy | governed input, command exit status, and retained evidence | command execution without the required baseline or policy file |

## Supporting Evidence Packages

Two private support packages strengthen proofs without becoming product owners:

| package | responsibility | start here |
| --- | --- | --- |
| policy support | executable repository-shape and governance guardrails | [policy support guide](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-policies/README.md) |
| scientific test support | reusable truth packets, fixtures, and reference-model helpers | [scientific test support guide](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-testkit/README.md) |

Policy support checks repository conventions; it does not define GNSS product
behavior. Scientific test support helps produce or package independent
evidence; it does not make an implementation authoritative.

## Handbook Contents

The root contains one handbook for each primary owner:

- [Command workflows](bijux-gnss/index.md)
- [Shared GNSS contracts](bijux-gnss-core/index.md)
- [Repository infrastructure](bijux-gnss-infra/index.md)
- [Navigation science](bijux-gnss-nav/index.md)
- [Receiver execution](bijux-gnss-receiver/index.md)
- [Signal processing](bijux-gnss-signal/index.md)
- [Maintainer tooling](bijux-gnss-dev/index.md)

Handbook directory names match durable package ownership so published URLs,
repository links, and crate identities remain aligned. Link text should
describe the reader's question or the owning domain.

## When Sources Disagree

Documentation can lag implementation. Resolve a disagreement by inspecting:

1. the owning package's public facade and manifest;
2. the implementation that makes the disputed decision;
3. the narrowest test or independent reference that exercises it;
4. emitted artifacts and diagnostics when runtime state matters;
5. the package changelog for an intentional contract change.

Then correct the stale page. Do not preserve a handbook statement merely
because several other pages copied it.
