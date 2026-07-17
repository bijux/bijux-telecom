---
title: Review Checklist
audience: mixed
type: checklist
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Review Checklist

- Is the new or changed concept truly cross-crate meaning rather than one
  owner’s convenience?
- If `api.rs` changed, was the public export reviewed as a contract change?
- If serialized meaning changed, were docs and validation tests updated
  together?
- If an invariant moved, was the new downstream assumption written down?
- Did the change avoid importing higher-level crate behavior into core?
- Would a reader know which contract family owns the change without reading the
  full implementation diff?
