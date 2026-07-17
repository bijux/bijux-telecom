# Reporting

`bijux-gnss-policies` owns both failing guardrail checks and read-only structural reporting.

## Read-only reporting surface

`src/bin/purity_report.rs` currently reports:

- crate names discovered under `crates/`
- dependency counts
- presence of selected heavy dependencies
- public-item distribution between `api.rs` and non-API files
- feature inventory

## Why this matters

Not every structural concern should immediately be a hard failure. Some things need to be visible
first so maintainers can inspect drift, compare crates, and decide whether a new guardrail belongs
in the enforced set.

## Boundary rule

This reporting surface is descriptive, not mutating. If a future report wants to rewrite files or
"fix" crates, it no longer belongs in this binary without a deliberate boundary change.
