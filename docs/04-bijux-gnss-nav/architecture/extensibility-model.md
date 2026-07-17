---
title: Extensibility Model
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# Extensibility Model

Extending `bijux-gnss-nav` should mean adding a new scientific family or
deepening an existing one, not bolting new convenience glue onto the public
surface.

## Legitimate Extension Paths

- add a new constellation-specific format or orbit family under `formats/` or
  `orbits/`
- add a new correction family with clear GNSS-domain semantics
- extend `position`, `ppp`, or `rtk` with evidence-backed solver behavior
- add a new typed provider or model seam needed by multiple scientific paths

## Illegitimate Extension Paths

- adding command-specific wrappers directly to `api.rs`
- adding file-discovery logic to a parser family
- exposing internal solver helpers only because one caller needs quick access

## Review Question

Can the new surface be named by a durable scientific responsibility? If not, it
probably does not belong here yet.
