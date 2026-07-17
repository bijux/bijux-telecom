---
title: Estimation Contracts
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# Estimation Contracts

Estimation contracts are the broadest public family in `bijux-gnss-nav`, and
they need the strongest review discipline.

## Major Public Families

- EKF state and measurement-model primitives
- position solution, refusal, DOP, weighting, smoothing, and integrity surfaces
- RAIM detection, exclusion, and solution-separation evidence
- runtime-neutral navigation engine and filter configuration
- PPP configuration, lifecycle, product policy, and solution epochs
- RTK differencing, ambiguity fixing, baseline solving, and quality evidence
- advanced solution-claim and downgrade reporting

## What Makes These Contracts Sensitive

- higher-level crates genuinely depend on them
- many of these types encode scientific policy, not only data shape
- widening or narrowing one export can silently change what downstream owners
  are allowed to assume about navigation behavior

## Review Rule

Before changing a public estimation type, ask whether the change affects only
implementation freedom or the meaning of a public scientific claim.

## Closest Proof

- `crates/bijux-gnss-nav/src/estimation/`
- `crates/bijux-gnss-nav/docs/ESTIMATION.md`
- `crates/bijux-gnss-nav/docs/PUBLIC_API.md`
