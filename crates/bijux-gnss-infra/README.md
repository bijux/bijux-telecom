# bijux-gnss-infra

`bijux-gnss-infra` owns repository-side infrastructure: dataset registry,
run identity, persisted artifact layout, provenance hashing, receiver-profile
overrides, and experiment sweep expansion.

Start here when a reader needs to understand how an input becomes a governed
repository object or how a command writes reviewable evidence to disk. Do not
start here for acquisition science, signal-code math, navigation estimation, or
operator report wording.

## Reader Route

| question | go next |
| --- | --- |
| Which dataset metadata is trusted? | [Dataset guide](docs/DATASETS.md), `src/datasets/` |
| Where should a run write files? | [Run layout guide](docs/RUN_LAYOUT.md), `src/run_layout/` |
| How is provenance or hashing computed? | [Hashing guide](docs/HASHING.md), `src/hash/` |
| How do overrides and sweeps expand? | [Override guide](docs/OVERRIDES.md), [Experiment guide](docs/EXPERIMENTS.md) |
| What changed in this package? | [Package changelog](CHANGELOG.md) |

## Owned Boundary

- dataset registration and metadata interpretation
- deterministic run-directory layout and manifest persistence
- artifact inspection and validation adapters
- receiver-profile overrides and experiment sweep expansion
- provenance hashing helpers for repository-owned inputs and outputs

This crate does not own receiver execution algorithms, signal generation,
navigation estimation, or operator-facing report language.

```mermaid
flowchart LR
    input["dataset or profile"]
    registry["registry and overrides"]
    run["run layout"]
    artifact["manifest and artifacts"]
    review["review evidence"]

    input --> registry
    registry --> run
    run --> artifact
    artifact --> review
```

## Source Map

- `src/datasets/` owns dataset registry and raw-IQ sidecar metadata.
- `src/run_layout/` owns run identity, directories, paths, persistence, and
  records.
- `src/artifact_inspection/` owns inspection summaries and validation adapters.
- `src/overrides/` and `src/sweep.rs` own profile override and experiment sweep
  behavior.
- `src/hash/` owns provenance hashing helpers.
- `src/validate_reference.rs` owns infrastructure-side validation adapters.

## Documentation Map

- [Architecture guide](docs/ARCHITECTURE.md)
- [Boundary guide](docs/BOUNDARY.md)
- [Contract guide](docs/CONTRACTS.md)
- [Dataset guide](docs/DATASETS.md)
- [Experiment guide](docs/EXPERIMENTS.md)
- [Hashing guide](docs/HASHING.md)
- [Override guide](docs/OVERRIDES.md)
- [Public API](docs/PUBLIC_API.md)
- [Run layout guide](docs/RUN_LAYOUT.md)
- [Test guide](docs/TESTS.md)
- [Validation guide](docs/VALIDATION.md)

## Verification Focus

Use infra tests when changing repository semantics:

```sh
cargo test -p bijux-gnss-infra --test integration_overrides
cargo test -p bijux-gnss-infra --test integration_guardrails
```

Repository-wide lanes and package routing are documented in the
[workspace README](../../README.md).
