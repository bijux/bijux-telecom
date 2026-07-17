---
title: Local Development
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Local Development

## Working Habits

- read the matching crate-local docs before changing a command path
- identify the governed inputs and outputs before editing code
- trace the selected workflow from `Commands` to the specific `run_*` function
- keep edits grouped by one maintainer workflow instead of mixing unrelated
  governance surfaces

## Local Focus

This crate rewards narrow work. A small CLI or validation change can alter
repository policy behavior, so local development should stay precise even when
the code edit itself is short.

## First Proof Check

Inspect `crates/bijux-gnss-dev/docs/ARCHITECTURE.md`,
`crates/bijux-gnss-dev/docs/WORKFLOWS.md`, and
`crates/bijux-gnss-dev/docs/TESTS.md` first. Then trace the target workflow in
`crates/bijux-gnss-dev/src/main.rs` so local work stays aligned with actual
repository-facing effects.
