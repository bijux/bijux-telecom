---
title: Quality
audience: mixed
type: index
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Quality

Open this section when the question is whether `bijux-gnss-core` is proving
its contract claims strongly enough: invariants, tests, limitations, risk, and
review posture.

Quality is especially important in a foundational crate because green tests
alone can hide contract drift until every downstream crate has already adapted
to it.

## First Proof Check

- `crates/bijux-gnss-core/docs/INVARIANTS.md`
- `crates/bijux-gnss-core/docs/TESTS.md`
- `crates/bijux-gnss-core/tests/`

## Leave This Section When

- leave for [Foundation](../foundation/) when the doubt is still about what
  belongs in the crate
- leave for [Interfaces](../interfaces/) when the real question is what public
  promise exists, not how well it is defended
