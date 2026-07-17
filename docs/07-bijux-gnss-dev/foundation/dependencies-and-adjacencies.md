---
title: Dependencies And Adjacencies
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Dependencies And Adjacencies

This page explains which dependencies are legitimate for a maintainer-only
binary and which adjacent crates create the real review pressure.

## Direct Dependencies

- `clap` for the binary command contract
- `anyhow` for maintainer-facing error propagation
- `regex` for controlled parsing of benchmark output
- `toml` for reading reviewed governance files

## Adjacencies That Matter More Than The Cargo Graph

- `bijux-gnss-policies` is the closest support crate: it owns reusable
  repository guardrails while `bijux-gnss-dev` owns command workflows and
  maintenance evidence
- `bijux-gnss` is the public operator boundary whose product commands must not
  leak into maintainer-only automation
- `bijux-gnss-receiver`, `bijux-gnss-nav`, and `bijux-gnss-signal` are product
  crates whose benchmarks or evidence may be inspected here without
  transferring ownership
- `bijux-gnss-testkit` can supply deterministic shared truth for maintenance
  evidence, but should not become the maintainer-workflow owner

## Dependency Rules

- new dependencies are acceptable when they strengthen maintainer command
  clarity, governed-file validation, or deterministic evidence handling
- new dependencies are suspect when they pull product crates, broad frameworks,
  or incidental repository scripting behavior into this binary
- this crate should stay adjacent to repository workflows, not coupled to
  product logic or uncontrolled side effects

## Review Question

For every dependency or governed file addition, ask whether it strengthens a
durable maintainer boundary or whether it is attempting to smuggle unrelated
repository behavior into the dev crate.

## First Proof Check

Inspect `crates/bijux-gnss-dev/Cargo.toml`,
`crates/bijux-gnss-dev/docs/ARCHITECTURE.md`, and
`crates/bijux-gnss-dev/docs/GOVERNANCE_FILES.md`. Then inspect
`crates/bijux-gnss-dev/src/main.rs` and
`crates/bijux-gnss-dev/tests/integration_guardrails.rs` to confirm the binary
still depends only on general-purpose tooling while reading named governed
repository inputs.

## First Neighbor Proof Check

When a dependency question turns into reusable repository policy, inspect the
support-crate route in [../index.md](../index.md) and then the
`bijux-gnss-policies` crate surfaces. When it turns into product-command
ownership, inspect [01-bijux-gnss/foundation/dependencies-and-adjacencies](../../01-bijux-gnss/foundation/dependencies-and-adjacencies.md).
