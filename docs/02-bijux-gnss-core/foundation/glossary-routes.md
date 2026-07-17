---
title: Glossary Routes
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Glossary Routes

This page keeps the repository glossary close to the shared meanings owner
without duplicating every crate-local technical manual.

## Terms That Usually Start Here

- GNSS, PRN, and signal identity vocabulary
- observation-layer terms such as pseudorange, carrier phase, and Doppler
- shared physical and geometric vocabulary such as ECEF, ENU, cycles, and
  meters

## Routing Rule

If a glossary question is really about a shared record or physical unit, stay
in `bijux-gnss-core`. If it becomes about one signal family, one runtime loop,
or one estimator, hand off to the owning crate handbook.

## First Proof Check

- `crates/bijux-gnss-core/docs/CONTRACT_MAP.md`
- `../../06-bijux-gnss-signal/index.md`
- `../../05-bijux-gnss-receiver/index.md`
- `../../04-bijux-gnss-nav/index.md`
