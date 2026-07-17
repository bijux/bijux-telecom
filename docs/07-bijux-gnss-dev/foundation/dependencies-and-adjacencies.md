---
title: Dependencies And Adjacencies
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Dependencies And Adjacencies

## Direct Dependencies

- `clap` for the binary command contract
- `anyhow` for maintainer-facing error propagation
- `regex` for controlled parsing of benchmark output
- `toml` for reading reviewed governance files

## Adjacency Rules

- new dependencies are acceptable when they strengthen maintainer command
  clarity or governed-file validation
- new dependencies are suspect when they pull product crates or broad framework
  behavior into this binary
- this crate should stay adjacent to repository workflows, not coupled to
  product logic

## Review Question

For every dependency or governed file addition, ask whether it strengthens a
durable maintainer boundary or whether it is attempting to smuggle unrelated
repository behavior into the dev crate.
