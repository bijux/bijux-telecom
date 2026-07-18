---
title: Domain Language
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# Domain Language

This page fixes the durable vocabulary used across the navigation handbook.

## Orbit State

Typed satellite position, velocity, clock, and uncertainty information derived
from broadcast ephemeris or precise products.

## Navigation Product

A file or message family whose payload encodes GNSS navigation truth rather
than generic repository metadata. Examples include LNAV, CNAV, FNAV, INAV,
SP3, CLK, ANTEX, and bias SINEX.

## Correction Law

Reusable GNSS-domain computations such as ionosphere, troposphere, code and
phase bias application, group delay handling, and signal combinations.

## Estimator Behavior

The rules by which position, integrity, PPP, or RTK solvers interpret
observations and evidence to produce or refuse a solution.

## Runtime-Neutral

Behavior that depends on scientific inputs and configuration, but not on the
live scheduling, channel ownership, or operator transport used by downstream
packages.

## Time Interpretation

GNSS-system-specific handling of weeks, rollovers, offsets, and civil-time
relationships that exceed the foundational time contracts in `bijux-gnss-core`.
