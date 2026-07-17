---
title: Correction Contracts
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# Correction Contracts

Correction contracts define the reusable GNSS-domain computations that sit
between raw observations and final solution behavior.

## Owned Surfaces

- atmosphere configuration and context
- code and phase bias providers
- group-delay conversions
- broadcast-ionosphere residual summaries
- ionosphere-free, geometry-free, narrow-lane, and measured-ionosphere helpers
- phase-windup and signal-combination helpers

## Contract Rule

These helpers may depend on navigation-specific physical assumptions, but they
should remain reusable across solvers and callers. If a correction only makes
sense because one runtime loop happens to need it, it is probably not a public
navigation contract.

## Closest Proof

- `crates/bijux-gnss-nav/src/corrections/`
- `crates/bijux-gnss-nav/docs/CORRECTIONS.md`

## Protecting Proof

Inspect `crates/bijux-gnss-nav/src/corrections/`,
`crates/bijux-gnss-nav/docs/CORRECTIONS.md`, and the correction-focused tests
such as ionosphere, windup, and bias integration families to confirm these
contracts remain reusable navigation science rather than caller-specific
workarounds.
