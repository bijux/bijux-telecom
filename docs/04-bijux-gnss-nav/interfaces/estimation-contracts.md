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

- EKF state and measurement-model primitives in `src/estimation/ekf/`
- position solution, refusal, DOP, weighting, smoothing, and integrity
  surfaces in `src/estimation/position/`
- RAIM detection, exclusion, and solution-separation evidence in
  `src/estimation/position/raim.rs` and the surrounding integrity family
- runtime-neutral navigation engine and filter configuration in
  `src/estimation/position/navigation.rs` and `navigation_filter.rs`
- PPP configuration, lifecycle, product policy, and solution epochs in
  `src/estimation/ppp/`
- RTK differencing, ambiguity fixing, baseline solving, and quality evidence
  in `src/estimation/rtk/`
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
- `crates/bijux-gnss-nav/tests/integration_position.rs`
- `crates/bijux-gnss-nav/tests/integration_public_ppp_convergence.rs`
- `crates/bijux-gnss-nav/tests/integration_rtk_ambiguity_fixing.rs`
- `crates/bijux-gnss-nav/docs/ESTIMATION.md`
- `crates/bijux-gnss-nav/docs/PUBLIC_API.md`

## Protecting Proof

Inspect the estimation source family above together with position, PPP, RTK,
and integrity-focused tests before changing any public estimation type. Those
proofs show whether a change preserves implementation freedom or alters the
meaning of a public scientific claim.
