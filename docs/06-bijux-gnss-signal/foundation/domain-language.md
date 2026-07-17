---
title: Domain Language
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Domain Language

Use this vocabulary consistently when changing `bijux-gnss-signal`.

## Durable Terms

- signal registry:
  the canonical inventory of supported signal definitions and component layout
- code family:
  the constellation-specific implementation of primary and secondary code
  behavior
- DSP primitive:
  reusable signal math that does not assume one receiver runtime
- raw-IQ contract:
  the typed metadata and quantization vocabulary needed to interpret capture
  bytes
- signal-layer validation:
  checks that observations are structurally compatible at the signal boundary

## Terms To Avoid Blurring

- runtime:
  use this for behavior owned by `bijux-gnss-receiver`, not any function that
  happens to process samples
- artifact:
  use this for persisted or emitted outputs, not internal vectors or spectra
- navigation quality:
  use this only for `bijux-gnss-nav` or receiver-level judgments, not signal
  compatibility checks

## First Proof Check

Inspect `crates/bijux-gnss-signal/docs/CATALOG.md`,
`crates/bijux-gnss-signal/docs/CODE_FAMILIES.md`,
`crates/bijux-gnss-signal/docs/DSP.md`, and
`crates/bijux-gnss-signal/docs/VALIDATION.md`. Then inspect
`crates/bijux-gnss-signal/src/catalog.rs`,
`crates/bijux-gnss-signal/src/dsp/mod.rs`, and
`crates/bijux-gnss-signal/src/obs_validation.rs` to make sure the words on
this page still match the actual source ownership seams.
