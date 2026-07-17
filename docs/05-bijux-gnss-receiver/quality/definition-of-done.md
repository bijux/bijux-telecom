---
title: Definition Of Done
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Definition Of Done

A receiver documentation or code change is done when the runtime intent is
complete, reviewable, and honestly proven.

## Done Means

- the owning runtime family is explicit
- the handbook still describes the changed boundary or contract correctly
- the relevant local proof has been run or the gap is explicitly called out
- public runtime meaning has not changed silently
- the commit boundary matches one durable runtime intent

## Protecting Proof

- `crates/bijux-gnss-receiver/docs/TESTS.md`
- `crates/bijux-gnss-receiver/docs/RUNTIME.md`
- `crates/bijux-gnss-receiver/docs/REFERENCE_VALIDATION.md`
