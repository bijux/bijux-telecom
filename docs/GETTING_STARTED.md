# Getting Started

This is the canonical ingest-first path for `bijux-gnss`. It uses a deterministic raw IQ fixture to
prove metadata-backed ingest before stronger acquisition, tracking, and positioning claims.

## 1) Validate the Raw IQ Metadata

```bash
bijux gnss validate-sidecar --sidecar-file datasets/synthetic/demo.sidecar.toml --unregistered-dataset --output artifacts/basic_sidecar
```

## 2) Inspect the Registered Raw IQ Fixture

```bash
bijux gnss inspect --dataset demo_synthetic --output artifacts/basic_ingest
```

## 3) Run a Streaming Ingest Smoke Path

```bash
bijux gnss run --dataset demo_synthetic --output artifacts/basic_stream
```

## 4) Summarize Diagnostics

```bash
bijux gnss diagnostics summarize artifacts/basic_stream
```

## 5) Explore Outputs

Artifacts are written under the selected output directory and include:
- `manifest.json`
- `signal_quality_report.json` for raw-IQ workflows
- `summary.json`
- command-specific artifact directories when the invoked workflow emits them

`demo_synthetic` is an ingest fixture, not a navigation-truth corpus. Use it to verify explicit raw
IQ handling first, then move to stronger GNSS validation datasets for acquisition, tracking, and PVT.

The signal-quality report captures sample rate, IF, usable duration, DC offset, I/Q imbalance,
clipping, RMS, and an estimated noise floor for the analyzed input window.

## 6) Acquire PRNs from the Public Live-Sky Excerpt

```bash
bijux gnss acquire --dataset gps_l1_2022_03_27_excerpt --config configs/receiver_live_sky_gps_l1.toml --prn 11,12,25,31,32 --report json --output artifacts/live_sky_acquire
```

This dataset is a redistributed excerpt from Daniel Estévez's public GPS L1 recording on Zenodo.
Use `datasets/recorded/gps_l1_2022_03_27_excerpt.provenance.md` for source, license, and local
excerpt details.

## 7) Export a Deterministic Synthetic Truth Bundle

```bash
bijux gnss export-synthetic-iq --scenario configs/scenarios/synthetic_iq_reference.toml --report json --out artifacts/synthetic_iq_reference
```

This command emits:
- `artifacts/synthetic_iq_reference/artifacts/synthetic_iq_reference.iq16`
- `artifacts/synthetic_iq_reference/artifacts/synthetic_iq_reference.sidecar.toml`
- `artifacts/synthetic_iq_reference/artifacts/synthetic_iq_reference.truth.json`
- `artifacts/synthetic_iq_reference/artifacts/synthetic_iq_reference.scenario.toml`

Use this bundle when you need deterministic raw IQ plus explicit PRN, Doppler, code phase, C/N0,
and navigation-bit truth for downstream ingest and acquisition checks.

## 8) Validate Injected Synthetic C/N0 Against Receiver Measurements

```bash
bijux gnss export-synthetic-iq --scenario configs/scenarios/synthetic_iq_cn0_reference.toml --report json --out artifacts/synthetic_iq_cn0_reference
bijux gnss validate-synthetic-iq --unregistered-dataset --file artifacts/synthetic_iq_cn0_reference/artifacts/synthetic_iq_cn0_reference.iq16 --sidecar artifacts/synthetic_iq_cn0_reference/artifacts/synthetic_iq_cn0_reference.sidecar.toml --truth artifacts/synthetic_iq_cn0_reference/artifacts/synthetic_iq_cn0_reference.truth.json --config configs/receiver_low_rate.toml --report json --out artifacts/synthetic_iq_cn0_validation
```

This calibration flow checks that measured prompt-power C/N0 remains within a configured tolerance
of the injected truth recorded in the synthetic bundle. The reference scenario uses one satellite by
design so C/N0 validation stays isolated from later tracking and multi-satellite integration work.
