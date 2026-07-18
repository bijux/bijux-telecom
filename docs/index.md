---
title: Repository Handbook
audience: mixed
type: index
status: canonical
owner: bijux-telecom-docs
last_reviewed: 2026-07-18
---

# Repository Handbook

`bijux-telecom` is a Rust GNSS workspace. The root handbook has one job:
route a reader to the crate that owns the next claim they need to trust. It is
not a second API manual and it should not make scientific claims that are only
proven inside one crate.

Start here when the question is still repository-level: which package owns a
behavior, which support crate carries the evidence, and where the first proof
surface lives. Leave this page as soon as the question becomes about one crate
API, one algorithm, one persisted artifact, or one test family.

<!-- bijux-telecom-badges:generated:start -->
[![License: Apache-2.0](https://img.shields.io/badge/license-Apache--2.0-0F766E)](https://github.com/bijux/bijux-telecom/blob/main/LICENSE)
[![CI](https://github.com/bijux/bijux-telecom/actions/workflows/ci.yml/badge.svg)](https://github.com/bijux/bijux-telecom/actions/workflows/ci.yml)
[![deploy-docs](https://img.shields.io/badge/deploy--docs-no%20status-9CA3AF)](https://github.com/bijux/bijux-telecom/actions/workflows/deploy-docs.yml)
[![release-crates](https://img.shields.io/badge/release--crates-no%20status-9CA3AF)](https://github.com/bijux/bijux-telecom/actions/workflows/release-crates.yml)
[![release-pypi](https://img.shields.io/badge/release--pypi-no%20status-9CA3AF)](https://github.com/bijux/bijux-telecom/actions/workflows/release-pypi.yml)
[![release-ghcr](https://img.shields.io/badge/release--ghcr-no%20status-9CA3AF)](https://github.com/bijux/bijux-telecom/actions/workflows/release-ghcr.yml)
[![release-github](https://img.shields.io/badge/release--github-no%20status-9CA3AF)](https://github.com/bijux/bijux-telecom/actions/workflows/release-github.yml)
[![release](https://img.shields.io/badge/release-no%20status-9CA3AF)](https://github.com/bijux/bijux-telecom/releases)
[![ghcr](https://img.shields.io/badge/ghcr-no%20status-9CA3AF)](https://github.com/bijux?tab=packages&repo_name=bijux-telecom)
[![published packages](https://img.shields.io/badge/published%20packages-no%20status-9CA3AF)](https://github.com/bijux/bijux-telecom)

[![Repository docs](https://img.shields.io/badge/docs-no%20status-9CA3AF?logo=materialformkdocs&logoColor=white)](https://github.com/bijux/bijux-telecom/tree/main/docs)
<!-- bijux-telecom-badges:generated:end -->

## Fast Route

| reader question | handbook | first proof after the handbook |
| --- | --- | --- |
| How does an operator command map to work? | [Command handbook](01-bijux-gnss/) | [Command source](../crates/bijux-gnss/src/cli/) and [command docs](../crates/bijux-gnss/docs/) |
| Which shared type, unit, time, diagnostic, or artifact meaning is canonical? | [Core handbook](02-bijux-gnss-core/) | [Core source](../crates/bijux-gnss-core/src/) and [core docs](../crates/bijux-gnss-core/docs/) |
| How are datasets, run identity, overrides, and persisted evidence interpreted? | [Infra handbook](03-bijux-gnss-infra/) | [Infra source](../crates/bijux-gnss-infra/src/) and [infra docs](../crates/bijux-gnss-infra/docs/) |
| Which navigation format, correction, orbit, or estimator owns the science? | [Navigation handbook](04-bijux-gnss-nav/) | [Navigation source](../crates/bijux-gnss-nav/src/) and [navigation docs](../crates/bijux-gnss-nav/docs/) |
| How does a receiver run stage acquisition, tracking, observations, and runtime validation? | [Receiver handbook](05-bijux-gnss-receiver/) | [Receiver source](../crates/bijux-gnss-receiver/src/) and [receiver docs](../crates/bijux-gnss-receiver/docs/) |
| Which signal catalog, code family, raw-IQ contract, or DSP primitive is reusable? | [Signal handbook](06-bijux-gnss-signal/) | [Signal source](../crates/bijux-gnss-signal/src/) and [signal docs](../crates/bijux-gnss-signal/docs/) |
| Which maintainer command governs audits, deny policy, benchmarks, or suite selection? | [Maintainer handbook](07-bijux-gnss-dev/) | [Maintainer command source](../crates/bijux-gnss-dev/src/main.rs) and [maintainer docs](../crates/bijux-gnss-dev/docs/) |
| What changed at workspace or package level? | [Workspace changelog](../CHANGELOG.md), package changelogs | `crates/<package>/CHANGELOG.md` |

```mermaid
flowchart LR
    reader["reader question"]
    root["docs/index.md<br/>package routing only"]
    owner["owning package handbook"]
    proof["crate README, crate docs,<br/>source, tests, artifacts"]
    support["support proof<br/>policies or testkit"]

    reader --> root
    root --> owner
    owner --> proof
    root --> support
```

## Package Chain

The installed GNSS command begins in `bijux-gnss`. That command may assemble
repository inputs through `bijux-gnss-infra`, stage execution through
`bijux-gnss-receiver`, consume signal truth from `bijux-gnss-signal`, consume
navigation science from `bijux-gnss-nav`, and exchange shared records through
`bijux-gnss-core`. Maintainer-only validation and benchmark workflows belong
to `bijux-gnss-dev`.

The chain is directional for documentation purposes:

- command questions start in the command owner, not in the crate that performs
  the deepest calculation
- shared records start in core, even when the value was emitted by receiver or
  navigation code
- repository evidence starts in infra, even when the run was launched by the
  command crate
- runtime-stage behavior starts in receiver, even when signal or navigation
  primitives are used inside the stage
- signal math starts in signal, even when the first visible failure is a
  receiver test
- navigation science starts in nav, even when the product is rendered by the
  command crate
- maintainer governance starts in dev, not in whichever workflow failed last

## Support Crates

Two crates are intentionally outside the seven handbook directories but still
carry critical proof:

| support crate | owns | inspect first |
| --- | --- | --- |
| `bijux-gnss-policies` | executable repository-shape and governance guardrails | [Policy crate README](../crates/bijux-gnss-policies/README.md) and [policy docs](../crates/bijux-gnss-policies/docs/) |
| `bijux-gnss-testkit` | reusable scientific fixtures, truth packets, and reference-model support | [Testkit crate README](../crates/bijux-gnss-testkit/README.md) and [testkit docs](../crates/bijux-gnss-testkit/docs/) |

Leave the seven-handbook chain when the strongest proof depends on one of
those support crates. Do not copy their claims into the root handbook as if the
root owns them.

## Root Contract

The root `docs/` tree owns exactly:

- [Command handbook](01-bijux-gnss/)
- [Core handbook](02-bijux-gnss-core/)
- [Infra handbook](03-bijux-gnss-infra/)
- [Navigation handbook](04-bijux-gnss-nav/)
- [Receiver handbook](05-bijux-gnss-receiver/)
- [Signal handbook](06-bijux-gnss-signal/)
- [Maintainer handbook](07-bijux-gnss-dev/)
- [Badge catalog](badges.md)
- this index

That structure is a reader contract. Extra root handbook directories should
not appear unless the repository adds another primary owner with the same
status as the seven package handbooks.

## What The Root Does Not Promise

- It does not document every public function. Use crate `docs/`, `src/`, and
  generated Rust documentation for API detail.
- It does not certify scientific truth by itself. Use tests, fixtures, and
  artifact evidence.
- It does not replace crate READMEs. Use the root to pick the owner, then read
  the owner.
- It does not flatten support crates into product owners. Policy and testkit
  claims must stay with their support crates.

## First Proof Check

Before trusting a repository-level claim, inspect these surfaces together:

- [Public landing page](../README.md) for the public landing claim
- [Workspace changelog](../CHANGELOG.md) for unreleased workspace-level changes
- [Workspace manifest](../Cargo.toml) for workspace membership and feature shape
- [Maintained command entrypoints](../Makefile) for verification routes
- this [repository handbook](index.md) for package routing
- the owning crate README, crate changelog, crate `docs/`, source, and tests for any
  crate-owned claim

If this page and a crate-local proof surface disagree, trust the crate-local
proof first and fix the root route so the next reader does not inherit the
conflict.
