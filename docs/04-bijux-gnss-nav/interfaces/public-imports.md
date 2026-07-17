---
title: Public Imports
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# Public Imports

This page summarizes the major import families published from
`bijux_gnss_nav::api`.

## Product Interpretation Imports

- broadcast navigation decoders for GPS, Galileo, BeiDou, and GLONASS
- RINEX navigation and observation parsing helpers
- precise-product surfaces such as SP3, CLK, ANTEX, and bias SINEX

## Orbit And Time Imports

- ephemeris and satellite-state types
- time-system conversion helpers and typed GNSS time records
- time-offset evidence and rollover interpretation support

## Correction Imports

- atmosphere configuration and correction context types
- code and phase bias provider seams
- ionosphere-free, geometry-free, narrow-lane, and measured-ionosphere helpers
- broadcast group-delay and broadcast-ionosphere residual families

## Estimation Imports

- EKF primitives and measurement-model seams
- position solution, integrity, RAIM, weighting, and runtime-neutral filtering
- PPP configuration, lifecycle, and product-policy surfaces
- RTK differencing, ambiguity, baseline, and quality surfaces
- advanced solution-claim and downgrade surfaces

## Reading Rule

If a consumer imports from an internal module path instead of this curated
surface, first check whether the API is missing a durable contract or whether
the caller is depending on internals by accident.
