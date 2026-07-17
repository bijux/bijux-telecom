---
title: Extensibility Model
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Extensibility Model

The crate should grow by adding new durable maintainer workflows, not by
turning into a generic automation toolbox.

## Honest Extension Paths

- add a new command when a repository-governance workflow needs typed reviewable
  ownership
- split `main.rs` by owned workflow family when the binary grows enough to make
  one-file ownership unclear
- add tests when a maintainer contract needs stronger structure proof

## Suspicious Extension Paths

- adding one-off shell convenience with no clear governed input or output
- introducing product behavior because the binary already runs from the
  repository root
- exporting Rust library helpers that encourage product crates to couple to
  maintainer internals

## Compatibility Discipline

Every extension should answer three questions:

- what reviewed maintainer workflow now exists
- what governed inputs or outputs it owns
- why it belongs in this binary rather than in product code or shared policy
