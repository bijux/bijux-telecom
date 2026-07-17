---
title: This Package Does Not Own
audience: mixed
type: boundary
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# This Package Does Not Own

This page is the refusal ledger for `bijux-gnss-dev`. When a proposed
maintainer command feels nearby but still belongs elsewhere, record the
decision here instead of blurring the boundary.

## Refusals

- operator-facing GNSS commands belong in `bijux-gnss`
- receiver runtime behavior and emitted product artifacts belong in
  `bijux-gnss-receiver`
- GNSS science and reusable product APIs belong in the owning product crates
- arbitrary repository scripting with no reviewed maintainer owner does not
  belong here
- hidden writes outside governed evidence locations do not belong here

## How To Use This Ledger

- add an entry when the same rejected design pressure keeps returning
- link the owning crate in review discussions
- prefer a clear refusal over a convenience command that nobody will own later
