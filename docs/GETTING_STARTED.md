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
- `summary.json`
- command-specific artifact directories when the invoked workflow emits them

`demo_synthetic` is an ingest fixture, not a navigation-truth corpus. Use it to verify explicit raw
IQ handling first, then move to stronger GNSS validation datasets for acquisition, tracking, and PVT.
