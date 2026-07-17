---
title: Error Model
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Error Model

The binary uses command-oriented failure rather than a shared library error
taxonomy.

## Error Families

- missing governed files
- malformed TOML or malformed reviewed fields
- expired governance entries
- failed external benchmark execution
- malformed benchmark output or baseline parsing

## Architectural Role

Errors here are maintainer-facing diagnostics. They should explain what
governed contract failed and why. They are not meant to become a reusable Rust
error API for downstream crates.
