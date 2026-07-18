# Bijux Rust Make Library

This directory provides reusable Rust Make targets and their governed shell
executors. It standardizes Cargo gate semantics and artifact placement while
leaving crate order, domain validation, release evidence, benchmarks, and
operations in each repository.

Load the library through the `rust` component in `bijux-makes`. See
[CONTRACT.md](CONTRACT.md) for target behavior and extension hooks.
