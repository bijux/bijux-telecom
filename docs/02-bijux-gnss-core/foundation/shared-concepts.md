---
title: Shared Concepts
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Shared Concepts

This page absorbs the old root-level concepts guide into the shared-contract
owner that actually defines the vocabulary lower crates exchange.

## Pipeline Concepts That Start Here

- acquisition:
  coarse visibility, code phase, and Doppler discovery
- tracking:
  maintained carrier and code lock with stable prompt/early/late semantics
- observations:
  the per-epoch measurement layer that higher crates consume
- navigation:
  the solution layer that interprets shared observations and models

## Why This Lives In Core

`bijux-gnss-core` should not own the implementation of each stage, but it does
own the shared record meanings that let signal, receiver, navigation, and
command crates speak consistently about them.

## First Proof Check

- `crates/bijux-gnss-core/docs/CONTRACT_MAP.md`
- `crates/bijux-gnss-core/src/observation/`
- `crates/bijux-gnss-core/src/nav_solution.rs`
