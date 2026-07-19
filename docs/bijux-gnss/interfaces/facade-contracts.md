---
title: Facade Contracts
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-18
---

# Facade Contracts

The Rust facade is intentionally smaller than the binary surface. It exists so
Rust consumers can discover the main GNSS stack through one crate without
turning the command crate into another owner of core, signal, receiver, or nav
semantics.

## Facade Shape

```mermaid
flowchart TD
    caller["Rust caller"]
    facade["bijux_gnss facade"]
    core["core"]
    signal["signal"]
    receiver["receiver"]
    nav["nav<br/>feature-gated"]

    caller --> facade
    facade --> core
    facade --> signal
    facade --> receiver
    facade --> nav
```

## Facade Commitments

| facade item | commitment | owner of behavior |
| --- | --- | --- |
| `core` re-export | shared records, units, diagnostics, artifacts, and time contracts remain reachable | `bijux-gnss-core` |
| `signal` re-export | signal definitions, code generation, sample contracts, and DSP helpers remain reachable | `bijux-gnss-signal` |
| `receiver` re-export | receiver stages, runtime helpers, ports, artifacts, and simulation contracts remain reachable | `bijux-gnss-receiver` |
| `nav` re-export | navigation APIs are available only when the feature is enabled | `bijux-gnss-nav` |

## Rejection Rules

- Do not add command-only helper functions to the facade.
- Do not re-export private lower-crate modules to avoid fixing a real public API
  gap in the owning crate.
- Do not use the facade to hide feature-gated navigation behavior.
- Do not add convenience aliases that make readers guess which lower crate owns
  the meaning.
- Do improve the owning crate first when an export has durable scientific or
  infrastructure meaning.

## First Proof Check

Inspect `crates/bijux-gnss/src/lib.rs`,
`crates/bijux-gnss/docs/FACADE.md`,
`crates/bijux-gnss/docs/PUBLIC_API.md`, and the `api.rs` files in the lower
crates before changing facade exports.
