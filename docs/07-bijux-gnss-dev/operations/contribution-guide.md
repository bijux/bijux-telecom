---
title: Contribution Guide
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-18
---

# Contribution Guide

Use this guide when changing repository-maintenance workflows, governed inputs,
or maintainer evidence. The goal is not to make every repository activity live
in `bijux-gnss-dev`; it is to keep important maintenance behavior typed,
reviewable, and bounded.

## Maintainer Change Flow

```mermaid
flowchart LR
    owner["confirm owner"]
    contract["name governed input<br/>command or output"]
    implementation["change typed workflow"]
    proof["run narrow verification"]
    docs["update owning docs"]
    commit["commit coherent intent"]

    owner --> contract --> implementation --> proof --> docs --> commit
```

## Contribution Gates

| gate | maintainer question | evidence |
| --- | --- | --- |
| ownership | Does this belong in maintainer tooling rather than product code or shared policy? | boundary docs |
| governed input | Which reviewed file or command contract is affected? | governance docs |
| output discipline | Where does evidence go, and is the location governed? | output docs |
| verification | Which narrow command proves the changed behavior? | test and workflow docs |
| reviewability | Can a reviewer see the changed contract without reading shell folklore? | docs and typed tests |

## Change Rules

- Keep each change inside one owning crate unless the contract truly crosses
  crate boundaries.
- Remove duplicate or thin structure when it obscures ownership.
- Use durable names for commands, files, outputs, and tests.
- Run the narrowest honest verification that backs the changed workflow.
- Commit one coherent maintainer intent at a time.

## First Proof Check

Inspect `crates/bijux-gnss-dev/docs/WORKFLOWS.md`,
`crates/bijux-gnss-dev/docs/TESTS.md`,
`crates/bijux-gnss-dev/docs/OUTPUTS.md`,
`crates/bijux-gnss-dev/docs/GOVERNANCE_FILES.md`,
`docs/07-bijux-gnss-dev/operations/verification-commands.md`, and the relevant
maintainer workflow tests.
