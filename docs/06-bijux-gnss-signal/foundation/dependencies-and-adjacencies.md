---
title: Dependencies And Adjacencies
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Dependencies And Adjacencies

## Direct Dependencies

- `bijux-gnss-core` for shared physical units, signal IDs, observation records,
  and sample container types
- `num-complex` for complex sample math
- `rustfft` for spectral analysis support
- `serde` and `schemars` for data-bearing contracts such as raw-IQ metadata and
  validation reports
- `thiserror` for signal-layer error reporting

## Adjacency Rules

- new dependencies are acceptable when they support reusable signal math or
  durable contract serialization
- new dependencies are suspect when they introduce filesystem I/O, operator
  policy, or receiver-runtime coupling
- higher-level crates may depend on `bijux-gnss-signal`; this crate must not
  depend on them

## Review Question

For every proposed dependency or export, ask whether it strengthens reusable
signal ownership or whether it is an attempt to sneak a higher-level concern
into the signal boundary.
