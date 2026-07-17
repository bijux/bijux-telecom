---
title: Integration Seams
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# Integration Seams

This page names the places where `bijux-gnss` intentionally meets neighboring
crates without surrendering ownership.

## Binary Surface Seam

`src/main.rs` is the public operator entrypoint. It should assemble stable
workflows over lower-level crates without duplicating their internals.

## Facade Seam

`src/lib.rs` is the Rust-package integration seam. It exists so consumers can
reach the GNSS stack through one package when they genuinely need that
convenience.

## Runtime And Repository Seams

- `command_runtime/` integrates runtime setup and repository-facing inspection
  helpers needed during command execution
- `command_support/` integrates lower-owner outputs into command workflows
  without transferring ownership

## Boundary Rule

Integration seams should pass operator inputs and lower-owner outputs. They
should not smuggle lower-owner science or repository policy into the command
crate.
