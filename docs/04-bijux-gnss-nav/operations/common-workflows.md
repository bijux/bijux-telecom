---
title: Common Workflows
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# Common Workflows

Use this page to route common navigation edits to the right scientific owner and
proof. Navigation work is reader-hostile when it lists files but hides the claim
being made, so every workflow below starts with the claim.

## Add Or Change A Format Family

- State the product being decoded or written: RINEX observation, RINEX
  navigation, SP3, CLK, ANTEX, bias SINEX, or a constellation navigation
  message.
- Keep repository discovery and persistence out of the parser.
- Update `crates/bijux-gnss-nav/docs/FORMATS.md` when public behavior changes.
- Prove the parser with representative input before relying on downstream
  workflow tests.

## Change Orbit Or Time Interpretation

- Identify whether the claim is constellation-local or shared across GPS,
  Galileo, BeiDou, or GLONASS.
- Check whether estimators consume the changed clock, orbit, or rollover value.
- Use reference-backed orbit and time tests; parser acceptance alone is not
  enough evidence.

## Change Correction Law

- Confirm the change is reusable navigation science, not a caller workaround.
- Keep physical models, signal combinations, atmosphere, bias, and tide logic in
  the correction or model owner that explains the formula.
- Run the specific correction test, then a position or estimator test only when
  the public solution changes.
- Review evidence fields and thresholds when the correction changes what a
  downstream report can claim.

## Change Estimator Behavior

- Identify whether the edit belongs to position, integrity, PPP, RTK, or EKF.
- Preserve refusal, downgrade, and uncertainty evidence as carefully as success
  outputs.
- Run the local solver proof first, then the smallest integration proof that
  shows the public effect.

## Change Public API

- Export by stable scientific role, not by internal file placement.
- Update `crates/bijux-gnss-nav/docs/PUBLIC_API.md` when downstream-facing names
  or semantics move.
- Do not expose a helper just because a command, receiver, or infra caller found
  it convenient; prove the role is navigation-owned.

## Proof Map

| workflow | primary docs | likely proof |
| --- | --- | --- |
| format decode or writer | `FORMATS.md` | product-specific parser or writer test |
| orbit, clock, or navigation time | `ORBITS.md`, `TIME.md` | constellation-specific orbit or rollover test |
| correction or physical model | `CORRECTIONS.md`, `MODELS.md` | numeric correction proof |
| estimator and integrity behavior | `ESTIMATION.md`, `TESTS.md` | solver, integrity, PPP, RTK, or EKF proof |
| downstream export | `PUBLIC_API.md`, `CONTRACTS.md` | API proof plus behavior proof |
