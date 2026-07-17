---
title: Fixture and Regression Care
audience: mixed
type: explanation
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Fixture and Regression Care

`bijux-gnss-core` has a small number of checked-in proof fixtures. They should
move rarely and deliberately.

## Main Persistent Proof Artifacts

- `tests/data/obs_fixture.jsonl`
- `tests/prop_timekeeping.proptest-regressions`

## Care Rules

- change a fixture only when serialized meaning or an invariant genuinely
  changes
- document why the old fixture stopped being correct
- keep the protecting test and the documentation move in the same change set

## Why This Matters

In a foundational crate, silent fixture churn can hide contract drift that
later looks like "normal" downstream adaptation.
