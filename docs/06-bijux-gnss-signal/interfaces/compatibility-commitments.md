---
title: Compatibility Commitments
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Compatibility Commitments

`bijux-gnss-signal` is consumed by multiple crates, so interface drift must be
treated as product drift.

## Commitments

- keep public behavior grouped by durable signal responsibility
- preserve one obvious public route through `bijux_gnss_signal::api`
- treat raw-IQ metadata, sample helpers, and validation reports as stable
  cross-crate contracts
- change or remove exports only with explicit proof that downstream ownership
  remains coherent

## Explicit Non-Commitments

- internal file layout outside the public facade is not a caller promise
- private lookup-table modules are not caller promises
- numeric implementation detail that is not exported is not a caller promise

## Protecting Proof

Inspect `crates/bijux-gnss-signal/docs/PUBLIC_API.md`,
`crates/bijux-gnss-signal/docs/CONTRACTS.md`, and
`crates/bijux-gnss-signal/src/api.rs`. Then inspect
`crates/bijux-gnss-signal/tests/integration_signal_component_registry.rs`,
`crates/bijux-gnss-signal/tests/integration_raw_iq_metadata.rs`, and
`crates/bijux-gnss-signal/tests/prop_obs_epoch_validation.rs` to confirm the
documented compatibility surface still has checked-in proof.
