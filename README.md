# bijux-gnss

`bijux-gnss` is a production-grade GNSS receiver stack in Rust: acquisition, tracking,
observations, navigation, and RTK/PPP scaffolding with reproducible artifacts and diagnostics.

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

## 30-Second Pitch
- Deterministic, testable pipeline from IQ → observations → PVT.
- Strict artifact contracts and schema validation.
- Research-ready diagnostics, experiments, and reproducibility hooks.

## Install

```bash
cargo build --workspace
```

## First Command

```bash
bijux gnss run --dataset demo_synthetic --output runs/basic_demo
```

Example output:
```
Artifacts: runs/basic_demo/artifacts
Manifest: runs/basic_demo/manifest.json
```

## Supported Features (Current)
- GPS L1 C/A acquisition + tracking
- Observation epoch generation
- PVT solver + RTK/PPP scaffolding
- Artifact validation + diagnostics summaries

## Maturity
Active development. Interfaces and outputs are stabilizing; use deterministic mode for
regression runs.

## MSRV
Minimum supported Rust version: `1.78.0`.

## Documentation
- `docs/README.md`
- `docs/GETTING_STARTED.md`
- `docs/CLI.md`
