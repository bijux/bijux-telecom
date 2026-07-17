---
title: Measurement And Engine Contracts
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Measurement And Engine Contracts

This page consolidates the old root-level measurement-model and engine-contract
guides into the shared-contract owner.

## Shared Contract Families

- `SamplesFrame` and related sample-domain bridge records
- observation records such as `ObsEpoch`, `ObsMetadata`, and timing alignment
  data
- artifact headers and envelope expectations that cross engine boundaries
- receiver-time, transmit-time, and measurement-sign conventions that higher
  crates must read consistently

## Why This Belongs In Core

The receiver, signal, navigation, and command crates may each touch these
records, but only `bijux-gnss-core` should define what the records mean across
those boundaries.
