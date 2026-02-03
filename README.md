# bijux-gnss

`bijux-gnss` is a production-grade GNSS receiver stack in Rust: acquisition, tracking,
observations, navigation, and RTK/PPP scaffolding with reproducible artifacts and diagnostics.

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
bijux gnss run --scenario configs/scenarios/basic.toml --output runs/basic_demo
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
