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
bijux gnss inspect --dataset demo_synthetic --output artifacts/basic_ingest
```

Example output:
```
Artifacts: artifacts/basic_ingest/artifacts
Manifest: artifacts/basic_ingest/manifest.json
```

`demo_synthetic` is a deterministic raw IQ ingest fixture. It proves explicit format, sample-rate,
IF, and capture-time handling; it is not a satellite-truth positioning dataset.

Raw-IQ workflows now also emit `signal_quality_report.json`, which records sample rate, IF,
usable duration, DC offset, I/Q imbalance, clipping, RMS, and an estimated noise floor for the
analyzed input window.

For a deterministic synthetic capture with machine-readable satellite truth, export the reference
scenario bundle:

```bash
bijux gnss export-synthetic-iq --scenario configs/scenarios/synthetic_iq_reference.toml --report json --out artifacts/synthetic_iq_reference
```

That command writes a raw IQ capture, a sidecar, a copied scenario file, and a truth JSON artifact
validated by `schemas/synthetic_iq_truth.schema.json`.

For deterministic C/N0 calibration against injected synthetic truth, export and validate the
single-satellite reference bundle:

```bash
bijux gnss export-synthetic-iq --scenario configs/scenarios/synthetic_iq_cn0_reference.toml --report json --out artifacts/synthetic_iq_cn0_reference
bijux gnss validate-synthetic-iq --unregistered-dataset --file artifacts/synthetic_iq_cn0_reference/artifacts/synthetic_iq_cn0_reference.iq16 --sidecar artifacts/synthetic_iq_cn0_reference/artifacts/synthetic_iq_cn0_reference.sidecar.toml --truth artifacts/synthetic_iq_cn0_reference/artifacts/synthetic_iq_cn0_reference.truth.json --config configs/receiver_low_rate.toml --report json --out artifacts/synthetic_iq_cn0_validation
```

This workflow verifies that measured prompt-power C/N0 stays within a configured tolerance of the
injected synthetic truth. The calibration scenario is intentionally single-satellite so the check
stays isolated from later multi-satellite and fractional-sampling validation work.

For a deterministic quantization-loss sweep against a float32 reference, measure the same
synthetic scenario across 16-bit, 8-bit, 4-bit, 2-bit, and 1-bit storage profiles:

```bash
bijux gnss measure-synthetic-quantization --scenario configs/scenarios/synthetic_iq_reference.toml --config configs/receiver_low_rate.toml --report json --out artifacts/synthetic_quantization_reference
```

This command emits `artifacts/synthetic_quantization_reference/artifacts/synthetic_quantization_loss_artifact.json`
with per-quantization acquisition-correlation loss and truth-guided tracking C/N0 loss against
the float32 baseline. When a profile does not yield a measurable acquisition peak, the artifact
records that loss as unbounded instead of fabricating a finite number.

For one final run-level GNSS accuracy artifact spanning acquisition through PVT, validate the
bundled synthetic navigation case:

```bash
bijux gnss validate-synthetic-navigation --scenario configs/scenarios/synthetic_navigation_accuracy.toml --config configs/receiver_low_rate.toml --report json --out artifacts/synthetic_navigation_accuracy
```

This command emits `artifacts/synthetic_navigation_accuracy/artifacts/gnss_accuracy_artifact.json`
with stage summaries, thresholds, pass/fail, data source, reference truth, and detailed
per-stage reports in one file.

For a public real-RF acquisition example, use the registered live-sky excerpt and its tuned
profile:

```bash
bijux gnss acquire --dataset gps_l1_2022_03_27_excerpt --config configs/receiver_live_sky_gps_l1.toml --prn 11,12,25,31,32 --report json --output artifacts/live_sky_acquire
```

The dataset provenance and redistribution details live in
`datasets/recorded/gps_l1_2022_03_27_excerpt.provenance.md`.

## Supported Features (Current)
- GPS L1 C/A acquisition + tracking
- Observation epoch generation
- Public RINEX GPS observation import with preserved L1/L2 observation channels
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
- `docs/TESTING.md`
- `makes/README.md`

## Workspace Layout
- `bijux-gnss` owns the public facade library and the `bijux` binary entrypoint, so top-level command wiring lives with the package users actually install.
- Domain crates such as `bijux-gnss-infra`, `bijux-gnss-nav`, and `bijux-gnss-receiver` own their production code and any crate-specific fuzzing surfaces.
- Fuzz harnesses live under `crates/<owner>/fuzz/` so parser and config fuzzing stays attached to the crate that owns the surface being exercised.
- `bijux-gnss-policies` remains a standalone crate because it is shared governance code consumed across the workspace.
- `bijux-gnss-testkit` remains a standalone crate because it provides reusable scientific reference models, fixtures, and truth data used by multiple GNSS crates.
