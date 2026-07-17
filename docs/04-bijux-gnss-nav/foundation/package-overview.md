---
title: Package Overview
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# Package Overview

`bijux-gnss-nav` is the scientific owner for GNSS navigation interpretation and
solution behavior in `bijux-telecom`.

## One-Sentence Role

This crate turns constellation-specific navigation products and measurements
into typed orbit state, correction evidence, and estimator outcomes that other
packages can trust.

## What Readers Should Remember

- formats here are not generic I/O; they are domain decoders for GNSS truth
- orbits, corrections, and estimation are kept together because they share
  physical assumptions and evidence models
- runtime orchestration stays outside this crate even when it consumes these
  solvers heavily

## Major Scientific Families

- `src/formats/` decodes navigation messages, RINEX products, and precise
  reference products such as SP3, CLK, ANTEX, and bias SINEX
- `src/orbits/` interprets broadcast and precise orbital state for GPS,
  Galileo, BeiDou, and GLONASS
- `src/corrections/` owns atmosphere, broadcast ionosphere, bias,
  dual-frequency, phase-windup, and signal-combination law
- `src/estimation/position/` owns PVT, integrity, runtime-neutral filter
  behavior, RAIM evidence, and solution smoothing
- `src/estimation/ppp/` owns precise point positioning state, lifecycle, and
  product-consumption policy
- `src/estimation/rtk/` owns differencing, ambiguity logic, baseline solving,
  and RTK quality evidence
- `src/time.rs` and `src/time/rollover.rs` own GNSS-specific time conversions
  and rollover interpretation above core
- `src/models/` owns supporting atmosphere, antenna, tide, celestial, and
  NeQuick surfaces needed by navigation computations

## Why This Package Is Heavy

The crate is large because the scientific boundary is large. A trustworthy
navigation solution is not only least squares or only product parsing. It is
the agreement between product interpretation, time-scale handling, correction
law, solver design, and integrity evidence.

## Closest Code Proof

- `crates/bijux-gnss-nav/src/api.rs`
- `crates/bijux-gnss-nav/src/formats.rs`
- `crates/bijux-gnss-nav/src/corrections/`
- `crates/bijux-gnss-nav/src/estimation.rs`
- `crates/bijux-gnss-nav/src/time.rs`
