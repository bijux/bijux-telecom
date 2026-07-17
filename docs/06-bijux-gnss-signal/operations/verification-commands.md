---
title: Verification Commands
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Verification Commands

Run the smallest honest proof set that matches the changed owner.

## Common Commands

```sh
cargo test -p bijux-gnss-signal --test integration_signal_component_registry
cargo test -p bijux-gnss-signal --test integration_signal_spectrum_cboc
cargo test -p bijux-gnss-signal --test integration_raw_iq_metadata
cargo test -p bijux-gnss-signal --test prop_obs_epoch_validation
```

## Matching Command To Change

- code-family changes:
  run the relevant reference and continuity tests for that family
- DSP changes:
  run the relevant long-duration, spectrum, or loop-behavior tests
- metadata or sample changes:
  run raw-IQ and sample-conversion tests
- validation changes:
  run observation-validation property and integration proof

## Bad Verification Pattern

Do not rely on one unrelated green test just because it exercises the crate.
This package has many proof families, and the changed owner should be obvious
from the verification set.
