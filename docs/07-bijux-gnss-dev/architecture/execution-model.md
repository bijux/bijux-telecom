---
title: Execution Model
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Execution Model

`bijux-gnss-dev` is an effectful maintainer binary. Reading repository files,
printing diagnostics, spawning benchmark commands, and writing evidence are the
point of the crate.

## Main Execution Shape

- parse one subcommand
- resolve the workspace root
- validate or derive behavior from reviewed inputs
- optionally execute benchmark commands
- emit diagnostics or maintenance evidence into governed locations

## Effect Rule

The binary is allowed to perform repository-scoped effects, but those effects
must remain explicit:

- what file is read
- what evidence file is written
- what external command is executed
- why the effect belongs to maintainer governance

## Why This Matters

The crate is not supposed to hide side effects behind a vague automation layer.
Its value comes from making maintainer behavior typed, reviewable, and bounded.
