# CLI Reference

This document is generated from the live `--help` output and provides examples for common flows.

## Top-Level Help

```
bijux-gnss CLI

Usage: bijux <COMMAND>

Commands:
  gnss  GNSS-related commands
  help  Print this message or the help of the given subcommand(s)

Options:
  -h, --help     Print help
  -V, --version  Print version
```

## GNSS Command Help

```
GNSS-related commands

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
  diagnostics         Diagnostics utilities
  validate-sidecar    Validate sidecar file against schema
  config-schema       Write JSON schema for receiver config
  config-upgrade      Upgrade a receiver config to the current schema version
  run                 Run a streaming pipeline with optional replay rate
  rinex               Write RINEX-like observation and navigation files
  doctor              Print build and runtime diagnostics
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
