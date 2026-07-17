---
title: Signal Model Assumptions
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Signal Model Assumptions

This page replaces the old root-level model manifest. It records the durable
signal-layer assumptions that downstream crates may rely on today.

## Supported Reference Families

- GPS L1 C/A is modeled end to end for code generation, sampled replicas,
  synthetic captures, acquisition, and tracking-facing validation
- Galileo E1 is modeled for code generation, synthetic captures, and
  acquisition-oriented validation

## Reusable Assumptions

- GPS L1 C/A repeats on an exact 1023-chip boundary for PRNs 1 through 32
- sampled-code generation advances from exact code-rate to sample-rate
  progression so chunk boundaries do not introduce rounded samples-per-chip
  drift
- synthetic carrier generation and acquisition both interpret Doppler relative
  to the configured intermediate frequency
- synthetic GPS navigation data uses a deterministic 50 bps sign modulation
  when data modulation is enabled

## Proof Expectation

These assumptions are only trustworthy when they stay backed by signal tests
and reference catalogs. When a change weakens the evidence, update the quality
and operations handbooks before claiming broader support.

## Protecting Proof

Inspect `crates/bijux-gnss-signal/docs/CATALOG.md`,
`crates/bijux-gnss-signal/docs/CODE_FAMILIES.md`,
`crates/bijux-gnss-signal/docs/DSP.md`, and
`crates/bijux-gnss-signal/src/catalog.rs`. Then inspect
`crates/bijux-gnss-signal/tests/integration_ca_code_reference.rs`,
`crates/bijux-gnss-signal/tests/integration_ca_code_long_duration_chunks.rs`,
`crates/bijux-gnss-signal/tests/integration_galileo_e1b_reference.rs`, and
`crates/bijux-gnss-signal/tests/integration_gps_l2c_replica_model.rs` to
confirm these assumptions remain reference-backed rather than descriptive only.
