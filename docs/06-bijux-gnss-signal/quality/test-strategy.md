---
title: Test Strategy
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Test Strategy

`bijux-gnss-signal` needs layered proof because it publishes canonical signal
behavior that many higher-level crates reuse.

## Main Proof Layers

- registry and reference tests for GPS, Galileo, BeiDou, and GLONASS families
- long-duration continuity tests for code phase, carrier wipeoff, NCOs, and
  replicas
- spectrum and front-end tests for BPSK, CBOC, and filtered responses
- raw-IQ metadata and sample-conversion tests
- property tests for C/A code, NCO behavior, and observation validation

## Why Layers Matter

Reference tests prove canonical signal truth. Continuity tests prove chunk and
time stability. Property tests prove structural rules. One layer cannot replace
the others without creating blind spots.
