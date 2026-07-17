---
title: Configuration Contracts
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# Configuration Contracts

`bijux-gnss` owns the operator-facing configuration surface even when lower
crates own specific validation or execution semantics.

## Main Configuration Areas

- receiver capture configuration such as sample rate, IF, and quantization
- acquisition parameters such as Doppler search bounds and thresholds
- tracking parameters such as loop bandwidths and correlator spacing
- navigation solver settings and weighting controls when the public workflow
  exposes them

## Boundary Rule

The command crate owns how configuration is presented and validated for the
operator. Lower crates still own the actual runtime, infrastructure, signal, or
navigation behavior behind those settings.

## Strongest Proof Surfaces

- command-facing validation flows such as `validate-config`
- crate-local command docs and examples
- the relevant lower-crate contract pages once ownership has been routed
