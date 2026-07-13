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

By default, the acquisition Doppler search range and bin width come from the selected receiver
profile. Override them with `--doppler-search-hz` or `--doppler-step-hz` only for an explicit
one-off experiment. Valid profiles keep `doppler_search_hz` aligned to `doppler_step_hz` so the
search grid includes both `0 Hz` and the configured edge.

`acquire_report.json` also records the code-phase search contract for the run. For GPS L1 C/A, that
contract currently spans the full sampled code period with one-sample spacing, so the reported
`bin_count` matches `period_samples`.

If the winning Doppler row supports sub-bin refinement, the same report also includes the refined
`carrier_hz` alongside `coarse_carrier_hz`, `doppler_refinement_hz`, and
`doppler_refinement_bins` so the search-bin provenance remains visible.

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
of the injected truth recorded in the synthetic bundle, and it also checks that acquisition
recovers the clean synthetic code phase within the configured sample tolerance. The default
validation bounds are `4.0 dB-Hz` for C/N0, `2` samples for acquisition code phase, and `1`
acquisition bin for Doppler recovery.

The reference scenario uses one satellite by design so C/N0 validation stays isolated from later
tracking and multi-satellite integration work.

## 9) Emit One Final GNSS Accuracy Artifact Per Synthetic Navigation Run

```bash
bijux gnss validate-synthetic-navigation --scenario configs/scenarios/synthetic_navigation_accuracy.toml --config configs/receiver_low_rate.toml --report json --out artifacts/synthetic_navigation_accuracy
```

This workflow emits `artifacts/synthetic_navigation_accuracy/artifacts/gnss_accuracy_artifact.json`.
That file records acquisition, tracking, observation, and PVT stage summaries together with the
underlying per-stage reports, thresholds, pass/fail, data source, and reference truth.

## 10) Sweep Synthetic Receiver Experiments

```bash
bijux gnss experiment --scenario configs/scenarios/basic.toml --config configs/receiver_low_rate.toml --sweep acquisition.integration_ms=1,2 --out artifacts/experiment_basic
```

Each sweep point now executes through the same receiver acquisition, tracking, observation, and
navigation path used by streaming runs. Expect the per-run outputs under
`artifacts/experiment_basic/artifacts/run_000/` to match the receiver pipeline contract rather
than a separate experiment-only implementation.
