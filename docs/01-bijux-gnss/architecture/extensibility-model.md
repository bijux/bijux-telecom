---
title: Extensibility Model
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# Extensibility Model

Extending `bijux-gnss` should mean adding or deepening a real command workflow,
not bolting convenience glue onto the top of the stack.

## Legitimate Extension Paths

- add a new operator workflow under `commands/`
- deepen the command catalog with a durable new argument or command family
- add a new reporting surface for a real command-owned output need
- add a new command/runtime support adapter that remains clearly command-owned

## Illegitimate Extension Paths

- adding lower-owner algorithms directly to command modules
- embedding repository layout policy into reporting or support helpers
- widening the facade only to save one caller an import statement

## Review Question

Can the new surface be named by a durable command responsibility? If not, it
probably does not belong here yet.
