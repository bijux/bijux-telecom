---
title: Domain Language
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# Domain Language

This page fixes the durable vocabulary used across the `bijux-gnss` handbook.
Use these terms when writing command docs, changelog entries, and review notes.
They keep the reader from confusing command ownership with lower-crate
ownership.

## Command Boundary

The public operator-facing surface: command names, flags, workflows, and output
shape. If a user can invoke it or parse it from command output, it is command
boundary language.

## Workflow Composition

The act of wiring lower-level crates into one top-level command flow without
claiming ownership of their deeper behavior. The command may sequence a receiver
run; it does not therefore own acquisition, tracking, or navigation science.

## Runtime Setup

The command-owned preparation needed before lower-level runtime execution
begins, such as environment handling, input loading, report destination
selection, or handoff to the repository run layout.

## Reporting

Operator-facing rendering of success, failure, or diagnostic output owned by
the command crate. Reporting can summarize lower-crate evidence, but it must not
rewrite what that evidence means.

## Facade Export

A thin Rust re-export surface that lets callers reach the GNSS stack through
one package without turning the CLI crate into a mixed-responsibility library.

## Ambiguous Terms To Avoid

| avoid | use instead |
| --- | --- |
| pipeline owns this | command routes this, receiver owns runtime execution |
| artifact output | operator report, receiver artifact, or infra run footprint |
| validation failed | command validation failed, receiver validation failed, or infra validation failed |
| support logic | command runtime setup, facade export, or lower-crate owner |

## Proof Path

Start with [Command contracts](../interfaces/command-contracts.md) when naming
operator behavior, [Facade contracts](../interfaces/facade-contracts.md) when
naming Rust imports, and [Reporting contracts](../interfaces/reporting-contracts.md)
when naming output. If the term needs signal, navigation, receiver, infra, or
core authority, leave this handbook and read the owner directly.
