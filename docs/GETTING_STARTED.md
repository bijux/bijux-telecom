# Getting Started

This is the canonical “first run” path for `bijux-gnss`. It uses a synthetic dataset to keep the flow deterministic.

## 1) Run a Synthetic Scenario

```bash
bijux gnss run --dataset demo_synthetic --output runs/basic_demo
```

## 2) Validate Artifacts

```bash
bijux gnss artifact validate runs/basic_demo/artifacts/obs.jsonl
bijux gnss artifact validate runs/basic_demo/artifacts/track.jsonl
```

## 3) Summarize Diagnostics

```bash
bijux gnss diagnostics summarize runs/basic_demo
```

## 4) Inspect a Specific Artifact

```bash
bijux gnss artifact explain runs/basic_demo/artifacts/obs.jsonl
```

## 5) Explore Outputs

Artifacts are written under `runs/<timestamp>_<dataset>_<command>/artifacts` and include:
- observation epochs
- tracking epochs
- acquisition results
- navigation solutions (when enabled)

See `docs/ARTIFACTS.md` for details.
