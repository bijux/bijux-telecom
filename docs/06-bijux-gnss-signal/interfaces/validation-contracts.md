---
title: Validation Contracts
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Validation Contracts

The validation surface answers signal-compatibility questions and stops there.

## Published Validation Families

- `check_dual_frequency_observations`
- `check_inter_frequency_alignment`
- `supported_dual_frequency_band_pairs`
- `supported_dual_frequency_band_pairs_for_constellation`
- `validate_obs_epochs`

## Published Data Shapes

- `DualFrequencyObservationReport`
- `DualFrequencyObservationPair`
- `DualFrequencyPairStatus`
- `DualFrequencyPairIssue`
- `InterFrequencyAlignmentReport`
- `BandLagEvent`

## Boundary Rule

These contracts can report missing bands, invalid lock-state combinations, or
non-monotonic epoch shapes. They do not decide whether a receiver run or
navigation solution is scientifically acceptable overall.
