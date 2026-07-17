# Validation

`bijux-gnss-signal` owns signal-layer compatibility validation, not navigation-quality judgment.

## Validation responsibilities

The validation surface currently owns:

- dual-frequency observation compatibility checks
- inter-frequency alignment reporting
- supported band-pair inventory for signal-layer validation

## Boundary rule

This crate can say whether observations are signal-compatible. It should not decide whether a full
navigation solution is trustworthy or whether a receiver workflow is acceptable. Those judgments
belong in higher layers.
