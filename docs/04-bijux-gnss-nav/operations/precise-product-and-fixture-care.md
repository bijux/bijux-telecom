---
title: Precise Product And Fixture Care
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# Precise Product And Fixture Care

`bijux-gnss-nav` relies on public-data, golden, and precise-product fixtures
that carry scientific meaning, not only test convenience.

## Care Rules

- treat SP3, CLK, ANTEX, bias SINEX, and constellation-specific decoder
  fixtures as scientific evidence
- when changing a fixture-backed behavior, explain whether the code or the
  expectation was wrong
- avoid silently broadening tolerances to make a reference test green
- keep parser and solver fixtures tied to the families that own them

## Why This Matters

This crate proves many claims against known references. Loose fixture hygiene
can make the test surface look broad while weakening its scientific value.

## First Proof Check

- `crates/bijux-gnss-nav/docs/FORMATS.md`
- `crates/bijux-gnss-nav/docs/ORBITS.md`
- `crates/bijux-gnss-nav/tests/integration_sp3_reference_accuracy.rs`
- `crates/bijux-gnss-nav/tests/integration_clk_reference_accuracy.rs`
