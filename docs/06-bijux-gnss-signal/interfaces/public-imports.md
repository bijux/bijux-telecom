---
title: Public Imports
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Public Imports

The public export strategy is grouped by durable responsibility, not by source
file convenience.

## Export Groups

- catalog exports expose signal definitions, wavelength conversions, shared-path
  Doppler helpers, and signal registry lookup
- code exports expose one canonical implementation per supported signal family
- DSP exports expose reusable mathematical helpers and state types such as
  `Nco`, `LocalCodeModel`, `ReplicaCodeModel`, and tracking-loop structures
- contract exports expose `RawIqMetadata`, quantization enums, and validation
  reports

## Review Rule

Every new export should answer two questions:

- does the signal crate truly own this behavior
- is the export name stable enough to remain understandable without the source
  conversation that introduced it

## Protecting Proof

Inspect `crates/bijux-gnss-signal/docs/PUBLIC_API.md`,
`crates/bijux-gnss-signal/docs/TRAITS.md`, and
`crates/bijux-gnss-signal/src/api.rs`. Then inspect
`crates/bijux-gnss-signal/tests/integration_guardrails.rs`,
`crates/bijux-gnss-signal/tests/integration_signal_component_registry.rs`, and
`crates/bijux-gnss-signal/tests/integration_replica_continuity.rs` to confirm
the export groups still map to durable owned surfaces.
