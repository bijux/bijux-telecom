# Layers

This repository is structured as explicit layers. Each layer has a clear responsibility and must not depend on lower-level implementation details.

## CLI (bijux-gnss-cli)
- Responsibility: Argument parsing, UX, report formatting, orchestration
- Allowed deps: infra, receiver, nav, signal, core
- Must not: expose domain types as CLI-only structs

## Infra (bijux-gnss-infra)
- Responsibility: Run layout, manifests, dataset registry, hashing, sweep utilities
- Allowed deps: core, receiver (for profile serialization)
- Must not: perform signal processing or navigation math

## Receiver (bijux-gnss-receiver)
- Responsibility: Pipeline orchestration (ingest → acquire → track → obs), runtime policies
- Allowed deps: signal, core, nav
- Must not: implement navigation models or core contracts

## Nav (bijux-gnss-nav)
- Responsibility: Ephemeris, corrections, solvers, filters
- Allowed deps: core
- Must not: depend on receiver or CLI

## Signal (bijux-gnss-signal)
- Responsibility: DSP primitives, codes, NCOs, correlator helpers
- Allowed deps: core
- Must not: depend on nav or receiver

## Core (bijux-gnss-core)
- Responsibility: Shared contracts, time systems, IDs, artifacts, errors
- Allowed deps: minimal utility crates only
- Must not: depend on higher layers
