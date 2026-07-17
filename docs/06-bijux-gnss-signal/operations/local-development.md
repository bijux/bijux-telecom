---
title: Local Development
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Local Development

## Working Habits

- read the matching crate-local docs before changing a code family or DSP area
- trace the change through `src/api.rs` if the behavior is publicly exported
- open the matching integration tests early so the proof surface guides the
  implementation
- keep edits grouped by signal owner rather than mixing unrelated surfaces

## Local Focus

This crate rewards narrow work. A small edit in signal math can have wide
downstream effect, so local development should stay precise even when the code
change itself is short.

## First Proof Check

Inspect `crates/bijux-gnss-signal/docs/ARCHITECTURE.md` and
`crates/bijux-gnss-signal/docs/TESTS.md` first. Then trace the target change
through `crates/bijux-gnss-signal/src/api.rs` and the owning source module so
local work stays aligned with the actual exported boundary and proof family.
