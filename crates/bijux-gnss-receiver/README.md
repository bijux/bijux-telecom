# bijux-gnss-receiver

`bijux-gnss-receiver` owns receiver runtime behavior: configuration,
acquisition, tracking, observation generation, optional navigation-stage
execution, diagnostics, and receiver-owned artifacts.

Start here when the question is about how a receiver run behaves at runtime.
Do not start here for signal-code definitions, standalone navigation science,
repository persistence policy, or operator-facing command wording.

## Reader Route

| question | go next |
| --- | --- |
| How does a run move through stages? | [docs/PIPELINE.md](docs/PIPELINE.md), `src/pipeline/` |
| Which runtime knobs and defaults are supported? | [docs/RUNTIME.md](docs/RUNTIME.md), `src/engine/receiver_config.rs` |
| Which ports isolate clock, source, and sink behavior? | [docs/PORTS.md](docs/PORTS.md), `src/ports/`, `src/io/` |
| Which reports or artifacts come from receiver execution? | [docs/ARTIFACTS.md](docs/ARTIFACTS.md), `src/artifacts.rs`, `src/validation_report.rs` |
| What changed in this package? | [CHANGELOG.md](CHANGELOG.md) |

## Owned Boundary

- receiver configuration, defaults, validation, and runtime state
- acquisition, tracking, observation, and optional navigation orchestration
- channel state, lock state, diagnostics, CN0, uncertainty, and refusal evidence
- clock, sample-source, and artifact-sink ports
- receiver-boundary simulation and reference-validation helpers

This crate does not own repository persistence, operator workflow policy,
low-level signal-code generation, or standalone navigation science.

```mermaid
flowchart LR
    source["sample source"]
    acq["acquisition"]
    track["tracking"]
    obs["observations"]
    nav["optional navigation"]
    artifacts["receiver artifacts"]

    source --> acq
    acq --> track
    track --> obs
    obs --> nav
    track --> artifacts
    obs --> artifacts
    nav --> artifacts
```

## Source Map

- `src/engine/` owns runtime configuration, logging, metrics, signal selection,
  support matrices, and receiver composition.
- `src/pipeline/` owns staged acquisition, tracking, observations, and optional
  navigation flows.
- `src/io/` and `src/ports/` own source, sink, and clock abstractions.
- `src/artifacts.rs`, `src/validation_report.rs`, and related modules own
  runtime-side outputs.
- `src/sim/` owns synthetic receiver execution helpers.

## Documentation Map

- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)
- [docs/ARTIFACTS.md](docs/ARTIFACTS.md)
- [docs/BOUNDARY.md](docs/BOUNDARY.md)
- [docs/CONTRACTS.md](docs/CONTRACTS.md)
- [docs/PIPELINE.md](docs/PIPELINE.md)
- [docs/PORTS.md](docs/PORTS.md)
- [docs/PUBLIC_API.md](docs/PUBLIC_API.md)
- [docs/REFERENCE_VALIDATION.md](docs/REFERENCE_VALIDATION.md)
- [docs/RUNTIME.md](docs/RUNTIME.md)
- [docs/SIMULATION.md](docs/SIMULATION.md)
- [docs/TESTS.md](docs/TESTS.md)

## Verification Focus

Use receiver tests that match the changed stage before full-suite proof:

```sh
cargo test -p bijux-gnss-receiver --test integration_basic
cargo test -p bijux-gnss-receiver --test integration_receiver_support_matrix_inventory
cargo test -p bijux-gnss-receiver --test integration_navigation_pvt_accuracy_budget
```

Repository-wide lanes and package routing are documented in
[../../README.md](../../README.md).
