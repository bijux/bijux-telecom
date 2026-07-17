---
title: Repository Fit
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# Repository Fit

`bijux-gnss-nav` is the repository's scientific middle layer. It is not the
lowest shared contract crate and not the highest operator-facing surface.

## Why The Repository Needs This Layer

- GNSS science should be reusable across commands, runtime flows, and
  persisted-reference validation
- higher-level crates need one place where orbit, correction, and estimator
  meaning is already settled
- lower-level crates should not be forced to absorb constellation-specific
  navigation law that would make them less reusable

## How It Fits With Neighboring Handbooks

- [02-bijux-gnss-core](../../02-bijux-gnss-core/) explains shared contracts that
  this crate builds on
- [03-bijux-gnss-infra](../../03-bijux-gnss-infra/) explains repository-facing
  ownership that may consume navigation evidence but should not redefine it
- [05-bijux-gnss-receiver](../../05-bijux-gnss-receiver/) explains runtime
  orchestration that may call navigation solvers but should not own them

## The Fit To Defend

This crate should remain the place where scientific claims are explainable
without opening command code, repository persistence code, or live sample-loop
orchestration.
