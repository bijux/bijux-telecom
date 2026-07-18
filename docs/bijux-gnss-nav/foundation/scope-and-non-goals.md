---
title: Scope And Non-Goals
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# Scope And Non-Goals

This page states what `bijux-gnss-nav` is allowed to own and what it should
refuse even when the neighboring code depends on its outputs.

## In Scope

- navigation-message decoding and precise-product parsing
- broadcast and precise orbit interpretation
- GNSS-specific time conversion and rollover handling above `core`
- correction models tied to navigation-domain semantics
- runtime-neutral position, integrity, PPP, and RTK solving behavior
- supporting physical models required directly by those computations

## Explicit Non-Goals

- sample streaming, acquisition scheduling, and tracking stage ownership
- repository run layout, artifact storage policy, or dataset discovery
- operator command surfaces and CLI argument semantics
- generic shared primitives that have no navigation-specific meaning
- hiding solver-local policy inside downstream runtime wrappers

## Why These Refusals Matter

If this crate absorbs runtime or persistence behavior, scientific law becomes
harder to inspect. If it tries to own generic shared primitives, neighboring
packages lose a stable lower boundary.

## Strong Negative Test

If a change cannot be explained in terms of product interpretation, correction
law, estimator behavior, or navigation-specific timing, it probably belongs in
another crate.
