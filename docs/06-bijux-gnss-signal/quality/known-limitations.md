---
title: Known Limitations
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Known Limitations

This crate can be mathematically rich while still being limited in what it
should claim.

## Main Limitations

- it can prove signal truth and compatibility, not full receiver or navigation
  success
- its test surface is broad, which demands discipline when choosing the right
  proof slice
- its public facade is large enough that export creep must be watched
- reference data can become persuasive too quickly unless changes are explained
  with real technical reasoning
