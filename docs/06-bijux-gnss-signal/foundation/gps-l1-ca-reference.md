---
title: GPS L1 C/A Reference
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# GPS L1 C/A Reference

This page replaces the old root-level GPS L1 C/A note. It records the durable
signal facts that readers usually need before they inspect code, validation, or
receiver behavior.

## Signal Shape

- carrier frequency: 1575.42 MHz
- code family: C/A
- chip rate: 1.023 MHz
- code length: 1023 chips
- primary-code period: 1 ms

## Repository Ownership

- spreading-code generation and correlation helpers live in
  `crates/bijux-gnss-signal/src/codes/ca_code.rs`
- reusable sampling and replica logic live in
  `crates/bijux-gnss-signal/src/dsp/replica.rs` and
  `crates/bijux-gnss-signal/src/dsp/signal.rs`
- receiver acquisition and tracking defaults belong to
  `bijux-gnss-receiver`
- navigation decode and orbit use belong to `bijux-gnss-nav`

## Reader Rule

Use this page when the question is the stable signal profile itself. Leave for
`../interfaces/signal-model-assumptions.md` when the question becomes what the
repository currently proves about synthetic generation, correlation, or sampled
phase behavior.

## First Proof Check

- `crates/bijux-gnss-signal/src/codes/ca_code.rs`
- `crates/bijux-gnss-signal/tests/integration_ca_code_reference.rs`
- `crates/bijux-gnss-signal/tests/integration_ca_code_long_duration_phase.rs`
