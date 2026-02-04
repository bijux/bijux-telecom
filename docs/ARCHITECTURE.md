# Architecture

This document defines the high-level layering and boundary rules for bijux-gnss.

## Layers (Top to Bottom)
1. CLI
2. Infra tooling
3. Receiver pipeline
4. Signal processing
5. Core domain

Dependencies are allowed only “down” the stack:
- CLI → infra, receiver, core, signal, nav
- Infra → receiver, core, signal, nav (no CLI)
- Receiver → signal, core, nav (no infra/CLI)
- Signal → core (no receiver/infra/CLI)
- Core → standard library only

This rule keeps domain logic isolated, prevents circular dependencies, and makes the
execution boundary explicit.

## Public API Rule
Only `src/api.rs` is the public façade for each crate.
Everything else is `pub(crate)` unless explicitly re-exported from `api.rs`.

Guidelines:
- `api.rs` declares types/traits and re-exports stable items.
- Concrete logic lives in internal modules (pipeline/app/engine/etc.).
- No `pub use crate::module::*` wildcard exports.

## Receiver Boundary
Receiver runtime logic lives in internal modules (engine/pipeline).
Ports define the purity boundary between I/O and domain logic:
- `SampleSource`: input samples
- `ArtifactSink`: output artifacts
- `Clock`: time source

These interfaces prevent I/O details from leaking into domain code.
