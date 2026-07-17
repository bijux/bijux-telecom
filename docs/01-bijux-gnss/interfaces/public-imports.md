---
title: Public Imports
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# Public Imports

This page summarizes the major public surfaces published by `bijux-gnss`.

## Binary Surface

- the `bijux` executable
- stable command vocabulary
- operator-facing reporting and workflow output

## Facade Surface

- `core`
- `receiver`
- `signal`
- `nav` when the feature is enabled

These are package-level convenience re-exports, not ownership transfers into
the command crate.

## Reading Rule

If a Rust consumer wants a helper that is not obviously a lower-level crate
re-export or a command-boundary contract, first check whether it belongs in the
CLI crate at all.
