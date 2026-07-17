---
title: Code Navigation
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Code Navigation

Use this route when reading the crate or reviewing a command change.

## Recommended Path

1. Start at the `Commands` enum in `src/main.rs` to see the owned workflow
   inventory.
2. Read `main()` to confirm how dispatch enters one workflow path.
3. Read the selected `run_*` function for the command being reviewed.
4. Follow any helper functions that validate reviewed input or emit evidence.
5. Finish in the matching crate-local doc and the integration tests.

## Practical Shortcut

If the change touches benchmark behavior, read the benchmark output writing and
baseline comparison helpers before reading the CLI flag handling. For audit or
deviation changes, read the governed-field validation loops first.
