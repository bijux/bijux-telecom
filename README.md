# bijux-gnss

`bijux-gnss` is a production-grade GNSS receiver stack in Rust: acquisition, tracking,
observations, navigation, and RTK/PPP scaffolding with reproducible artifacts and diagnostics.

<!-- bijux-telecom-badges:generated:start -->
[![License: Apache-2.0](https://img.shields.io/badge/license-Apache--2.0-0F766E)](https://github.com/bijux/bijux-telecom/blob/main/LICENSE)
[![Standards](https://github.com/bijux/bijux-telecom/actions/workflows/bijux-std.yml/badge.svg)](https://github.com/bijux/bijux-telecom/actions/workflows/bijux-std.yml)
[![Docs](https://github.com/bijux/bijux-telecom/actions/workflows/deploy-docs.yml/badge.svg)](https://github.com/bijux/bijux-telecom/actions/workflows/deploy-docs.yml)
[![Crates Publish](https://github.com/bijux/bijux-telecom/actions/workflows/release-crates.yml/badge.svg)](https://github.com/bijux/bijux-telecom/actions/workflows/release-crates.yml)
[![GitHub Release](https://github.com/bijux/bijux-telecom/actions/workflows/release-github.yml/badge.svg)](https://github.com/bijux/bijux-telecom/actions/workflows/release-github.yml)

[![crates.io status](https://img.shields.io/badge/crates.io-not%20published-F59E0B)](https://crates.io/)
[![GHCR status](https://img.shields.io/badge/ghcr-not%20published-F59E0B)](https://github.com/bijux?tab=packages&repo_name=bijux-telecom)
[![GitHub releases status](https://img.shields.io/badge/github%20releases-not%20published-F59E0B)](https://github.com/bijux/bijux-telecom/releases)

[![Repository docs](https://img.shields.io/badge/docs-repository-2563EB?logo=materialformkdocs&logoColor=white)](https://github.com/bijux/bijux-telecom/tree/main/docs)
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
