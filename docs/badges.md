---
title: Badge Catalog
audience: maintainer
type: reference
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# Badge Catalog

This page is the shared badge contract for `bijux-gnss` public entry
surfaces. It is the single source of truth for the generated badge block used
by the repository entry pages.

Keep the generated block below identical in:

- the root `README.md`
- the repository [handbook landing page](index.md)

Badge order is part of the contract:

1. repository summary and workflow badges
2. crates.io package badges in release-contract order
3. GHCR package badges in the same order
4. repository and Rust API documentation badges

Link policy is also fixed here:

- workflow badges must link to the exact workflow file or release surface they
  summarize
- every crates.io badge must link to that exact crate page
- every GHCR badge must link to the corresponding repository package
- documentation badges must link to the checked-in handbook or docs.rs API
  surface they name

Review rule:

- if a badge link changes, update both entry surfaces in the same change set
- if a badge is removed, explain which repository promise no longer deserves a
  public summary badge
- if a new badge is added, it must represent a durable repository surface, not
  a short-lived rollout state

## First Proof Check

Inspect the root README, the [handbook landing page](index.md), and the
generated badge block below together. The proof for this page is not just the
Markdown here; it is that the repository's public entry surfaces still expose
one identical badge contract.

## Reader-Facing Rule

Badges are not decoration. They are short claims about durable repository
surfaces. If a badge would need a paragraph of caveats to stay honest, the
badge should change or disappear instead of training readers to distrust the
entry surfaces.

The badge count must agree with the six-package allowlist in the
[crate release contract](../configs/release/crates.toml). Repository-only
crates do not receive crates.io or GHCR badges.

## Generated Badge Block

<!-- bijux-gnss-badges:generated:start -->
[![License: Apache-2.0](https://img.shields.io/badge/license-Apache--2.0-0F766E)](https://github.com/bijux/bijux-gnss/blob/main/LICENSE)
[![CI](https://github.com/bijux/bijux-gnss/workflows/repo%20/%20ci/badge.svg)](https://github.com/bijux/bijux-gnss/actions/workflows/ci.yml?query=branch%3Amain)
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
