---
title: Code Navigation
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# Code Navigation

Use this page when you know the question but not the owning file.

## Start From The Question

- "How is this navigation product parsed?":
  `src/formats/` and the matching constellation, RINEX, or precise-product
  subfamily
- "How is orbit state computed?":
  `src/orbits/`
- "How is this correction derived?":
  `src/corrections/`, especially `combinations.rs`, `dual_frequency.rs`, or
  the matching single-purpose correction file
- "Why did the position solution refuse or downgrade?":
  `src/estimation/position/`, `src/estimation/solution_claims.rs`
- "Why did PPP or RTK behave this way?":
  `src/estimation/ppp/` or `src/estimation/rtk/`
- "Why did a time conversion or rollover behave this way?":
  `src/time.rs` and `src/time/rollover.rs`

## Start From A Public Export

- begin in `src/api.rs`
- locate the re-exported symbol family
- move into the owning submodule only after confirming the export is public on
  purpose

## Start From A Test Failure

- parser and product failures usually map to one family in
  `crates/bijux-gnss-nav/tests/` with the same constellation or product name
- estimator failures usually map to the
  `integration_position*`, `integration_ppp*`, `integration_rtk*`, or
  `integration_raim*` families under `crates/bijux-gnss-nav/tests/`
