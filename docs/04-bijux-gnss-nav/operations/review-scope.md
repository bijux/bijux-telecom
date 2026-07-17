---
title: Review Scope
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# Review Scope

Reviewers should scope a nav change by scientific family first, then by file
count.

## Narrow Review

- one decoder family
- one correction family
- one estimator-local behavior inside position, PPP, or RTK
- one time or orbit rule with clearly bounded downstream effects

## Broad Review Triggers

- changes to `src/api.rs`
- changes that span formats and estimators at once
- changes to refusal, downgrade, integrity, or ambiguity-fix evidence
- changes that require synchronized edits in `receiver`, `infra`, or `gnss`

## Why File Count Is Misleading

A one-line threshold change in RAIM or PPP policy can be riskier than a larger
refactor isolated inside one parser family.
