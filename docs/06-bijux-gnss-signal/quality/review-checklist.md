---
title: Review Checklist
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Review Checklist

Review `bijux-gnss-signal` as reusable signal truth and DSP substrate. The
crate may own signal catalogs, code generation, raw-IQ and sample contracts,
runtime-neutral DSP primitives, and validation helpers. It should not own
receiver scheduling, persisted dataset policy, or navigation decisions.

```mermaid
flowchart TB
    change["signal-layer change"]
    kind{"catalog, code, sample, DSP, or validation?"}
    local["send back to receiver, nav, infra, or command owner"]
    proof["inspect signal contract and reference proof"]
    export["review public API only when the surface is reusable"]

    change --> kind
    kind -- no --> local
    kind -- yes --> proof --> export
```

## Review Gates

| changed surface | accept only when | inspect before accepting |
| --- | --- | --- |
| signal catalog or component metadata | The catalog remains canonical and does not duplicate receiver-local assumptions. | [Signal Model Assumptions](../interfaces/signal-model-assumptions.md), `crates/bijux-gnss-signal/tests/integration_signal_component_registry.rs` |
| spreading code or code-family behavior | The reference proof still explains the public signal fact being claimed. | [Code Contracts](../interfaces/code-contracts.md), [GPS L1 C/A Reference](../foundation/gps-l1-ca-reference.md) |
| raw-IQ or sample representation | Sample semantics stay runtime-neutral and do not encode ingestion policy. | [Raw IQ And Sample Contracts](../interfaces/raw-iq-and-sample-contracts.md), `crates/bijux-gnss-signal/docs/BOUNDARY.md` |
| DSP primitive | Continuity, normalization, and signal meaning are proven without assuming one receiver loop. | [DSP Contracts](../interfaces/dsp-contracts.md), `crates/bijux-gnss-signal/tests/integration_signal_spectrum_cboc.rs` |
| public export or trait | The surface is reusable across product owners and does not expose internal tables as API. | [API Surface](../interfaces/api-surface.md), [Trait Contracts](../interfaces/trait-contracts.md), `crates/bijux-gnss-signal/tests/integration_guardrails.rs` |

## Blocking Signs

- A receiver failure is fixed by adding receiver policy to the signal crate.
- A new public export only saves one caller an import or exposes a lookup table
  that should remain internal.
- Reference data changes without a clear signal-model reason.
- A DSP test proves numerical convenience but not the signal contract readers
  will rely on.

## Evidence To Require

- Read `crates/bijux-gnss-signal/docs/PUBLIC_API.md`,
  `crates/bijux-gnss-signal/docs/BOUNDARY.md`, and
  `crates/bijux-gnss-signal/docs/TESTS.md` before accepting public or broad
  signal changes.
- Require the narrow proof family for the changed surface: catalog, code, raw
  samples, DSP, trait, or validation.
- Update interface or foundation docs when public signal meaning changes.
- Route runtime state, repository layout, and navigation judgment to their
  owners instead of broadening signal to host them.
