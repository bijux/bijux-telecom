---
title: Badge Catalog
audience: maintainer
type: reference
status: canonical
owner: bijux-telecom-docs
last_reviewed: 2026-07-17
---

# Badge Catalog

`docs/badges.md` is the shared badge contract for `bijux-telecom` public entry
surfaces. It is the single source of truth for the generated badge block used
by the repository entry pages.

Keep the generated block below identical in:

- the root `README.md`
- the repository handbook landing page at `docs/index.md`

This repository does not yet publish the richer templated badge system used in
some other Bijux repositories. Until that automation exists here, treat this
file as the single source of truth and keep the generated block in sync across
both entry surfaces instead of hand-editing badge lines independently.

Badge order is part of the contract:

1. repository summary and workflow badges
2. release and publication posture badges
3. repository documentation badge

Link policy is also fixed here:

- workflow badges must link to the exact workflow file or release surface they
  summarize
- the GHCR summary badge must link to
  `https://github.com/bijux?tab=packages&repo_name=bijux-telecom`
- the documentation badge must link to the checked-in `docs/` tree until a
  published docs site becomes canonical

If this repository later adopts templated badge automation, migrate that
automation from this file rather than letting `README.md` and `docs/index.md`
grow separate badge policies.

## Generated Badge Block

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
