---
title: Definition of Done
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Definition of Done

An infra change is done when the repository contract is still legible and
defended.

## Done Means

- ownership is still correct
- affected contract docs are aligned
- the narrow protecting tests were run when they apply
- uncovered areas are described honestly when dedicated proof is absent
- no product behavior was silently absorbed into infra

## Not Done Yet Means

- a new helper exists only because another owner did not want to hold it
- run-footprint meaning changed but the docs still describe the old behavior
- the repository trust story depends on assumptions that are no longer written
  down

## First Proof Check

Inspect `crates/bijux-gnss-infra/docs/CONTRACTS.md`,
`crates/bijux-gnss-infra/docs/TESTS.md`,
`crates/bijux-gnss-infra/docs/RUN_LAYOUT.md`, and the changed source family.
An infra change is not done until the repository contract prose and protecting
proof still describe the same behavior.
