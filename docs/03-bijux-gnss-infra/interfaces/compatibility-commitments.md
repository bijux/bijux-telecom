---
title: Compatibility Commitments
audience: mixed
type: explanation
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Compatibility Commitments

Compatibility in `bijux-gnss-infra` is primarily about repository-state
meaning.

## Commitments

- the curated infra API should stay intentional rather than accidental
- dataset interpretation rules should not drift silently
- run manifests, reports, and history semantics should change deliberately and
  with documentation
- typed override and sweep behavior should remain reviewable

## Non-Commitments

- private module layout is not a public promise
- re-export convenience is not a license for callers to treat lower-level APIs
  as infra-owned
- command UX and runtime policy are not part of infra compatibility

## Protecting Proof

- `crates/bijux-gnss-infra/docs/CONTRACTS.md`
- `crates/bijux-gnss-infra/docs/RUN_LAYOUT.md`
