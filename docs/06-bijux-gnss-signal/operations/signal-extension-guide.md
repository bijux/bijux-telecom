---
title: Signal Extension Guide
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Signal Extension Guide

This page replaces the old root-level signal-extension note. Add new signal
families here when the behavior is reusable substrate, not a receiver-only
shortcut.

## Extension Sequence

1. implement the code or modulation in the owning `codes/`, `catalog`, or
   `dsp/` area
2. expose it through the curated public API only if another crate genuinely
   needs it
3. add deterministic proof in the exact family test set, such as
   `integration_ca_code_reference.rs`,
   `integration_galileo_e5_reference.rs`,
   `integration_signal_spectrum_cboc.rs`, or the relevant long-duration
   continuity test
4. record any new durable model assumption in
   `../interfaces/signal-model-assumptions.md`

## Boundary Rule

If the work is really about receiver search policy, stage scheduling, or
tracking-state transitions, keep it in `bijux-gnss-receiver`. If it is about
navigation message interpretation or orbit products, keep it in
`bijux-gnss-nav`.

## First Proof Check

- `crates/bijux-gnss-signal/docs/CODE_FAMILIES.md`
- `crates/bijux-gnss-signal/docs/DSP.md`
- `crates/bijux-gnss-signal/docs/TESTS.md`
