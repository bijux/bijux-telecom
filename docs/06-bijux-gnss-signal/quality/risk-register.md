---
title: Risk Register
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Risk Register

## Main Risks

- a higher-level crate may pressure this package into owning runtime behavior
- a new signal family may be added with incomplete reference proof
- public re-exports may grow faster than the actual contract discipline behind
  them
- metadata or validation helpers may drift into policy rather than staying
  signal-layer contracts

## Mitigations

- keep the refusal ledger current
- review `api.rs` changes as boundary changes
- require family-specific proof for new signal additions
- keep validation focused on compatibility rather than broad trust judgment
