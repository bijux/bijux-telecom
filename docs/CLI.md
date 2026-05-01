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
  ca-code             Generate GPS L1 C/A code for a PRN
  acquire             Acquire satellites from a raw i16 sample file
  track               Track satellites from a raw i16 sample file
  nav                 Navigation-related commands
  pvt                 Solve PVT from a dataset
  inspect             Inspect dataset statistics
  rtk                 RTK alignment and SD/DD artifacts
  experiment          Run parameter sweeps over synthetic scenarios
  artifact            Artifact validation and conversion
  validate-config     Validate a receiver profile configuration file
  validate-artifacts  Validate observation or ephemeris artifacts against schemas
  diagnostics         Diagnostics and audit workflows for receiver evidence
  validate-sidecar    Validate sidecar file against schema
  config-schema       Write JSON schema for receiver config
  config-upgrade      Upgrade a receiver config to the current schema version
  run                 Run a streaming pipeline with optional replay rate
  rinex               Write RINEX-like observation and navigation files
  doctor              Print receiver runtime readiness diagnostics
  validate            Run a full validation pipeline and emit validation_report.json
  help                Print this message or the help of the given subcommand(s)

Options:
  -h, --help  Print help
```

## Common Examples

### Run Pipeline
```bash
bijux gnss run --dataset demo_synthetic --output runs/basic_demo
```

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
