---
title: Known Limitations
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Known Limitations

This crate is intentionally small, but its workflows still have limits.

## Main Limitations

- command-oriented validation can prove governance shape, not every downstream
  human review decision
- benchmark workflows can be expensive enough that full execution is not always
  practical in every local pass
- one-file binary organization is still workable today, but it demands
  discipline as commands grow
- maintainer convenience pressure can make bad workflow additions seem
  reasonable unless the boundary is defended explicitly

## First Proof Check

- `crates/bijux-gnss-dev/docs/BENCHMARKS.md`
- `crates/bijux-gnss-dev/docs/TESTS.md`
- `crates/bijux-gnss-dev/src/main.rs`
