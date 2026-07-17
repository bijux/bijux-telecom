---
title: Review Scope
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Review Scope

Reviewers should align their depth to the changed maintainer workflow.

## Scope By Change Type

- audit workflow change:
  inspect field requirements, expiry discipline, and derived-ignore behavior
- deviation workflow change:
  inspect ownership, review-link, and expiry requirements
- benchmark workflow change:
  inspect curated benchmark scope, evidence writing, normalization, and
  threshold comparison
- repository guardrail test change:
  inspect whether the new policy still belongs to maintainer governance

## Review Shortcut

If the diff changes which repository file is read or where evidence is written,
treat the change as a boundary review even when the code diff looks small.
