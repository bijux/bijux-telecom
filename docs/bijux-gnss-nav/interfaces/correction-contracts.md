---
title: Correction Contracts
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-18
---

# Correction Contracts

Correction contracts are reusable navigation science between observations and
solution estimates. They encode physical assumptions: atmosphere, ionosphere,
group delay, biases, signal combinations, and carrier effects.

They are public contracts only when more than one solver, product path, or
higher-level crate needs the same correction meaning.

## Correction Flow

```mermaid
flowchart LR
    obs["observations"]
    products["broadcast or precise products"]
    models["atmosphere and physical models"]
    corrections["correction values"]
    estimator["position, PPP, or RTK estimator"]

    obs --> corrections
    products --> corrections
    models --> corrections
    corrections --> estimator
```

## Contract Families

| family | owns | first proof |
| --- | --- | --- |
| atmosphere and ionosphere | atmospheric context, broadcast ionosphere, measured ionosphere, residual summaries | the [atmosphere model](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-nav/src/corrections/atmosphere.rs), [broadcast residual model](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-nav/src/corrections/broadcast_ionosphere_residuals.rs), and [measured ionosphere model](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-nav/src/corrections/measured_ionosphere.rs) |
| biases and group delay | code bias, phase bias, broadcast group-delay conversions | the [bias model](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-nav/src/corrections/biases.rs) and [broadcast group-delay model](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-nav/src/corrections/broadcast_group_delay.rs) |
| combinations | ionosphere-free, geometry-free, narrow-lane, Melbourne-Wubbena, and related combinations | the [combination model](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-nav/src/corrections/combinations.rs), [ionosphere-free code model](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-nav/src/corrections/iono_free_code.rs), [ionosphere-free phase model](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-nav/src/corrections/iono_free_phase.rs), and [Melbourne-Wubbena model](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-nav/src/corrections/melbourne_wubbena.rs) |
| carrier effects | phase windup and carrier-aware correction helpers | the [phase-windup model](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-nav/src/corrections/phase_windup.rs) |
| dual-frequency support | dual-frequency correction and diagnostic surfaces | the [dual-frequency correction model](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-nav/src/corrections/dual_frequency.rs) |

## Boundary Rules

- Navigation owns correction law and model assumptions.
- Signal owns carrier, wavelength, code, raw-IQ, and DSP behavior before
  navigation correction.
- Core owns shared observation and unit record meaning.
- Receiver owns when a correction is invoked during a runtime pipeline.
- Infra owns persisted product discovery and run evidence around correction
  use.

## Reader Checks

- Which physical assumption changed?
- Which observation, product, or model input is required?
- Does the correction remain reusable across PVT, PPP, RTK, or product
  validation?
- Does a higher-level failure need correction proof or only receiver/infra
  handoff proof?

## First Proof Check

Start with the navigation [correction guide](https://github.com/bijux/bijux-gnss/blob/main/crates/bijux-gnss-nav/docs/CORRECTIONS.md)
and the [correction source tree](https://github.com/bijux/bijux-gnss/tree/main/crates/bijux-gnss-nav/src/corrections).
Then run or inspect the correction-focused integration tests for ionosphere,
windup, bias, and signal-combination behavior before changing correction
claims.
