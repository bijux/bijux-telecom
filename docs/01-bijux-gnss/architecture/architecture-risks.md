---
title: Architecture Risks
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# Architecture Risks

This page records the main structural risks in `bijux-gnss`.

## Main Risks

- the crate sits at the top of the stack, so convenience pressure can slowly
  erase lower-owner boundaries
- included CLI modules in `main.rs` can make the command surface feel flatter
  than it really is
- support helpers can attract repository or runtime logic simply because they
  are nearby
- the Rust facade can quietly grow into a mixed-responsibility library
- operator reporting can start embedding lower-owner policy instead of only
  presenting it

## What To Watch In Review

- whether a new path is command-owned or merely caller-convenient
- whether a new export or helper is a durable public contract or only a
  shortcut
- whether a new family belongs under an existing command workflow rather than as
  a new peer at the top
