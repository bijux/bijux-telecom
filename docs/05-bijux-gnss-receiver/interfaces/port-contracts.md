---
title: Port Contracts
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Port Contracts

Port contracts define how the runtime touches the outside world without owning
repository or command policy.

## Owned Port Surfaces

- `SampleSource` and its adapters
- `ArtifactSink`
- `Clock` and `SystemClock`
- `FileSamples`, `MemorySamples`, and `SampleSourceError`

## Contract Rule

Ports are execution-oriented seams. They may carry runtime inputs and outputs,
but they should not become repositories for persisted naming policy or command
workflow assumptions.

## Closest Proof

- `crates/bijux-gnss-receiver/src/ports/`
- `crates/bijux-gnss-receiver/src/io/data.rs`
- `crates/bijux-gnss-receiver/docs/PORTS.md`
