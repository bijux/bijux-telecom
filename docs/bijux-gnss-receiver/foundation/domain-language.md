---
title: Domain Language
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Domain Language

This page fixes the durable vocabulary used across the receiver handbook.

## Runtime Composition

The act of turning configuration, ports, and stage engines into one executing
receiver run.

## Stage Handoff

The typed transition between acquisition, tracking, observations, and optional
navigation, including reports and runtime-side evidence.

## Port Seam

An execution-oriented trait or adapter for time, samples, or artifacts that
lets the runtime interact with the outside world without hard-coding a
repository or command policy.

## Receiver Artifact

The in-memory product of a receiver run before repository persistence or
indexing occurs.

## Runtime-Side Validation

Reference alignment, comparison, covariance realism, or validation reporting
that evaluates receiver outputs while staying at the runtime boundary rather
than the repository boundary.

## Synthetic Receiver Execution

Simulation helpers that exercise the receiver surface directly to prove runtime
behavior, stage sensitivity, or validation logic.
