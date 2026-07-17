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
3. Read the selected `run_*` function for the command being reviewed:
   `run_audit_allowlist_check`, `run_deny_policy_deviations_check`,
   `run_audit_ignore_args`, or `run_bench_compare`.
4. Follow the helper functions that validate reviewed input or emit evidence,
   especially `current_iso_day`, `is_iso_day`, `write_current_snapshot`, and
   `compare_baseline`.
5. Finish in the matching crate-local doc and the integration tests.

## Practical Shortcut

If the change touches benchmark behavior, read the benchmark output writing and
baseline comparison helpers before reading the CLI flag handling. For audit or
deviation changes, read the governed-field validation loops first. If the
question is about slow-test roster policy, leave `src/main.rs` and inspect
`tests/integration_nextest_suite_selection.rs`, because that contract is
test-governed rather than command-consumed.

## First Proof Check

Open `crates/bijux-gnss-dev/docs/ARCHITECTURE.md`,
`crates/bijux-gnss-dev/docs/WORKFLOWS.md`, and
`crates/bijux-gnss-dev/docs/TESTS.md` first. Then follow the route from
`crates/bijux-gnss-dev/src/main.rs` into the selected `run_*` function and the
matching integration test so code navigation stays aligned with owned
repository workflows.
