---
title: Execution Model
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# Execution Model

`bijux-gnss` is the package where operator intent becomes one top-level
workflow.

## Normal Flow

1. operator input enters through stable command names and flags
2. the command catalog and parsing layer select a workflow family
3. command runtime and support helpers prepare the environment and inputs
4. the selected command delegates execution to lower-level crates
5. the command boundary renders operator-facing output or validation results

## Important Architectural Distinction

The command crate owns workflow selection, orchestration, and reporting. It
does not own the reusable signal, receiver, or navigation behavior invoked
inside those workflows.

## Families With Separate Execution Logic

- diagnostics commands assemble evidence-heavy reporting flows
- validate commands assemble schema, artifact, and science-policy checks
- pipeline and synthetic commands assemble runtime execution over lower owners
- artifact and analysis commands assemble repository-facing reads without
  becoming the persistence owner
