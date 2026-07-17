---
title: Architecture Risks
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Architecture Risks

This page records the main structural risks in `bijux-gnss-receiver`.

## Main Risks

- the crate is broad enough that runtime convenience can slowly erase internal
  stage boundaries
- receiver-owned adapters over nav science can be mistaken for permission to
  own the science itself
- runtime artifacts can attract repository semantics because they are later
  persisted elsewhere
- synthetic helpers can sprawl into an informal truth system if not kept tied
  to receiver-boundary proof
- public re-exports can make lower-owner surfaces look receiver-owned

## What To Watch In Review

- whether a new path is runtime-owned or merely caller-convenient
- whether a new export is a durable receiver contract or only a shortcut
- whether a new family belongs under an existing subsystem rather than as a new
  peer at the top level
