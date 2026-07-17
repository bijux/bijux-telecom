# Execution

`bijux-gnss` owns operator-facing execution setup over lower-level GNSS crates.

## Execution responsibilities

The CLI execution surface currently owns:

- top-level argument interpretation before handing off to lower layers
- runtime setup and execution support under `src/cli/command_runtime.rs` and
  `src/cli/execution_support.rs`
- command-level sequencing of lower-level crate calls

## Boundary rule

This crate owns operator workflow execution, not the lower-level science and persistence semantics
that those workflows invoke.
