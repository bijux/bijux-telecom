# CLI Reference

This document is generated from the live `--help` output and provides examples for common flows.

## Top-Level Help

```
GNSS receiver workflows: run, inspect, validate, diagnose, replay, and compare

Usage: bijux <COMMAND>

Commands:
  gnss  GNSS receiver commands for operation, validation, diagnostics, and replay
  help  Print this message or the help of the given subcommand(s)

Options:
  -h, --help     Print help
  -V, --version  Print version
```

## GNSS Command Help

```
GNSS receiver commands for operation, validation, diagnostics, and replay

Usage: bijux gnss <COMMAND>

Commands:
  ca-code              Generate GPS L1 C/A code for a PRN
  acquire              Acquire satellites from a raw IQ file with explicit metadata
  track                Track satellites from a raw IQ file with explicit metadata
  nav                  Navigation-related commands
  pvt                  Solve PVT from a dataset
  inspect              Inspect raw IQ dataset statistics from explicit metadata
  rtk                  RTK alignment and SD/DD artifacts
  experiment           Run parameter sweeps over synthetic scenarios
  export-synthetic-iq  Export a deterministic synthetic raw IQ bundle and matching truth artifact
  validate-synthetic-iq  Validate a synthetic IQ bundle against truth-guided C/N0 expectations
  validate-synthetic-navigation  Validate a full synthetic navigation scenario from acquisition through PVT
  artifact             Artifact validation and conversion
  validate-config      Validate a receiver profile configuration file
  config               Configuration utilities
  validate-artifacts   Validate observation or ephemeris artifacts against schemas
  diagnostics          Diagnostics and audit workflows for receiver evidence
  validate-sidecar     Validate sidecar file against schema
  analyze              Analyze a GNSS run directory and emit evidence-oriented summaries
  diff                 Compare two GNSS run directories for quality deltas
  config-schema        Write JSON schema for receiver config
  config-upgrade       Upgrade a receiver config to the current schema version
  run                  Run a streaming pipeline with optional replay rate
  rinex                Write RINEX-like observation and navigation files
  doctor               Print receiver runtime readiness diagnostics
  validate             Run a full validation pipeline and emit validation_report.json
  validate-reference   Validate a run directory against a reference trajectory
  help                 Print this message or the help of the given subcommand(s)

Options:
  -h, --help  Print help
```

## Common Examples

### Run Pipeline
```bash
bijux gnss inspect --dataset demo_synthetic --output artifacts/basic_ingest
```

All raw-IQ workflows (`inspect`, `acquire`, `track`, and `run`) also emit
`signal_quality_report.json` in the selected run directory.

### Real RF Acquisition
```bash
bijux gnss acquire --dataset gps_l1_2022_03_27_excerpt --config configs/receiver_live_sky_gps_l1.toml --prn 11,12,25,31,32 --report json --output artifacts/live_sky_acquire
```

The effective Doppler grid comes from the receiver profile's `[acquisition]` section by default:
- `doppler_search_hz` is the maximum search range on each side of IF
- `doppler_step_hz` is the bin width
- `intermediate_freq_hz` defines the search center

Use `--doppler-search-hz` and `--doppler-step-hz` only when you need a one-off override for a
specific run. Valid receiver profiles must keep `doppler_search_hz` as an integer multiple of
`doppler_step_hz` so the search grid includes both `0 Hz` and the configured search edge.

The acquisition report also records the effective code-phase search scope. For GPS L1 C/A, the
current receiver searches the full sampled code period with one-sample spacing and emits that
contract under `code_phase_search` in `acquire_report.json`.

When acquisition can refine the Doppler estimate between search bins, `acquire_report.json`
preserves both values instead of hiding the raw search result:
- `carrier_hz` is the refined carrier estimate used downstream
- `coarse_carrier_hz` is the selected Doppler-bin carrier
- `doppler_refinement_hz` and `doppler_refinement_bins` report the bounded offset from the coarse bin

### Inspect Published C/A PRN Assignment
```bash
bijux gnss ca-code --prn 1 --start-chip 1022 --count 4 --with-reference
```

This prints the published G2 tap pair, G2 delay, period length, wrapped chip offset, first-10-chip octal reference, and the requested chip window across the period boundary.

### Inspect C/A Autocorrelation Summary
```bash
bijux gnss ca-code --prn 1 --count 4 --with-autocorrelation
```

This prints the validated periodic autocorrelation peak, the maximum absolute non-zero sidelobe magnitude, the distinct non-zero sidelobe values, and the requested chip window.

### Inspect C/A Cross-Correlation Summary
```bash
bijux gnss ca-code --prn 1 --count 4 --cross-correlation-prn 2
```

This prints the validated periodic cross-correlation PRN pair, the maximum absolute cross-correlation magnitude, the distinct cross-correlation values, and the requested chip window.

### Export Synthetic IQ Truth Bundle
```bash
bijux gnss export-synthetic-iq --scenario configs/scenarios/synthetic_iq_reference.toml --report json --out artifacts/synthetic_iq_reference
```

### Validate Synthetic IQ C/N0 Calibration
```bash
bijux gnss validate-synthetic-iq --unregistered-dataset --file artifacts/synthetic_iq_cn0_reference/artifacts/synthetic_iq_cn0_reference.iq16 --sidecar artifacts/synthetic_iq_cn0_reference/artifacts/synthetic_iq_cn0_reference.sidecar.toml --truth artifacts/synthetic_iq_cn0_reference/artifacts/synthetic_iq_cn0_reference.truth.json --config configs/receiver_low_rate.toml --report json --out artifacts/synthetic_iq_cn0_validation
```

This workflow validates two truth-guided properties from the same bundle:
- prompt-power C/N0 against the injected truth, with a default tolerance of `4.0 dB-Hz`
- acquisition code-phase recovery against clean regenerated truth, with a default tolerance of `2` samples
- acquisition Doppler recovery against clean regenerated truth, with a default tolerance of `1` Doppler bin

Use `--tolerance-db-hz` or `--acquisition-code-phase-tolerance-samples` when you need a stricter
or looser bound for a specific validation run. Use `--acquisition-doppler-tolerance-bins` when the
relevant scientific bound should be expressed in acquisition bins rather than raw Hz.

### Validate Synthetic Navigation Accuracy
```bash
bijux gnss validate-synthetic-navigation --scenario configs/scenarios/synthetic_navigation_accuracy.toml --config configs/receiver_low_rate.toml --report json --out artifacts/synthetic_navigation_accuracy
```

This workflow emits `artifacts/gnss_accuracy_artifact.json`, one machine-readable file per run
that carries acquisition, tracking, observation, and PVT summaries alongside detailed stage
reports, thresholds, pass/fail, data source, and reference truth.

### Validate Artifacts
```bash
bijux gnss artifact validate --file runs/basic_demo/artifacts/obs.jsonl
bijux gnss artifact explain --file runs/basic_demo/artifacts/obs.jsonl
```

### Diagnostics Summary
```bash
bijux gnss diagnostics summarize --run-dir runs/basic_demo
```

### Diagnostics Workflow Map
```bash
bijux gnss diagnostics workflow --report json
```

### Diagnostics Operator Map
```bash
bijux gnss diagnostics operator-map --report table
```

### Diagnostics Explain (Replay, Cache, Identity)
```bash
bijux gnss diagnostics explain --run-dir runs/basic_demo
```

### Diagnostics Reproducibility Verification
```bash
bijux gnss diagnostics verify-repro --run-dir runs/basic_demo
```

### Diagnostics Compare (Evidence + Quality)
```bash
bijux gnss diagnostics compare --baseline-run-dir runs/baseline --candidate-run-dir runs/candidate --report json
```

### Diagnostics Replay Audit
```bash
bijux gnss diagnostics replay-audit --baseline-run-dir runs/baseline --candidate-run-dir runs/candidate --report json
```

### Diagnostics Artifact Inventory
```bash
bijux gnss diagnostics artifact-inventory --run-dir runs/basic_demo --report json
```

### Diagnostics Debug Plan
```bash
bijux gnss diagnostics debug-plan --run-dir runs/basic_demo --report table
```

### Diagnostics Benchmark Summary
```bash
bijux gnss diagnostics benchmark-summary --run-dir runs/basic_demo --report json
```

### Diagnostics Medium Gate
```bash
bijux gnss diagnostics medium-gate --run-dir runs/basic_demo --strict --report json
```

### Diagnostics Operator Status
```bash
bijux gnss diagnostics operator-status --run-dir runs/basic_demo --report table
```

### Diagnostics Channel Summary
```bash
bijux gnss diagnostics channel-summary --run-dir runs/basic_demo --report json
```

### Diagnostics Export Bundle
```bash
bijux gnss diagnostics export-bundle --run-dir runs/basic_demo
```

### Diagnostics Machine Catalog
```bash
bijux gnss diagnostics machine-catalog --report json
```

### Advanced Claim Gate
```bash
bijux gnss diagnostics advanced-gate --run-dir runs/rtk_eval --mode rtk --strict --report json
```

### Validation Evidence Bundle
```bash
bijux gnss validate --file runs/basic_demo/artifacts/obs.jsonl --eph runs/basic_demo/artifacts/ephemeris.json --reference refs/reference.jsonl
# emits runs/.../artifacts/validation_report.json
# emits runs/.../artifacts/validation_evidence_bundle.json
```

Validation and diagnostics JSON reports include explicit claim-bound fields (`claim_evidence_guard`, `claim_level`, and `interpretation`) so strong claims are only made when recorded evidence supports them.
