---
title: Dependency Direction
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Dependency Direction

The dependency rule is narrow: general-purpose CLI and parsing tools enter from
below, reviewed repository files are consumed at the edge, and product crates
stay out.

## Inbound Direction

- `clap` supplies the command-line contract
- `anyhow` supplies command-oriented error propagation
- `regex` supports benchmark-line extraction
- `toml` supports governed-file parsing

## Internal Direction

- the command enum selects one owned workflow at a time
- helper functions are shaped around maintainer workflows, not around reusable
  library abstractions
- benchmark comparison depends on parsed benchmark output and governed file
  locations, not on product-crate internals

## Outbound Direction

- the crate reads reviewed files from the repository root
- the crate may execute `cargo bench` and `date`
- the crate writes governed maintenance evidence under repository-owned
  locations

## Prohibited Direction

This binary must not pull in product crates just to reach behavior indirectly.
If reusable product logic is needed, the owning product crate should expose it
first.

## First Proof Check

Inspect `crates/bijux-gnss-dev/Cargo.toml`,
`crates/bijux-gnss-dev/docs/BOUNDARY.md`, and
`crates/bijux-gnss-dev/docs/GOVERNANCE_FILES.md`. Then inspect
`crates/bijux-gnss-dev/src/main.rs` to confirm the binary still reads reviewed
files and emits governed evidence without depending on product crates.
