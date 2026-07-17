---
title: Change Principles
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Change Principles

Changes in `bijux-gnss-dev` shape how the repository governs itself, so
convenience is not a sufficient standard.

## Principles

- prefer one reviewed source of exception truth over duplicated automation
- keep maintainer effects explicit about what is read and what is written
- choose names by repository meaning, not by temporary enforcement history
- keep commands narrow and typed rather than growing a generic utility bucket
- document governed inputs and outputs whenever a maintainer workflow changes

## Anti-Patterns

- adding a command because a shell snippet feels repetitive without deciding who
  should own the workflow
- letting benchmark evidence drift into ad hoc locations
- introducing product behavior into the maintainer binary because it is already
  convenient to invoke from the repository root
