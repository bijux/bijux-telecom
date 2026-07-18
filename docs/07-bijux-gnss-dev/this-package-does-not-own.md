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

## Strongest Neighboring Owners

- [Command handbook](../01-bijux-gnss/) for public GNSS commands
- [Receiver handbook](../05-bijux-gnss-receiver/) for runtime behavior
  and emitted product artifacts
- [Infra handbook](../03-bijux-gnss-infra/) for persisted evidence and
  repository-facing run mechanics
- [Core handbook](../02-bijux-gnss-core/),
  [Navigation handbook](../04-bijux-gnss-nav/), and
  [Signal handbook](../06-bijux-gnss-signal/) for reusable GNSS product
  semantics that maintainer tooling should only inspect, never own

## Review Trigger

- add an entry when the same rejected design pressure keeps returning
- link the owning crate in review discussions
- prefer a clear refusal over a convenience command that nobody will own later

## First Neighbor Proof Check

- [Command handbook](../01-bijux-gnss/index.md)
- [Receiver handbook](../05-bijux-gnss-receiver/index.md)
- [Infra handbook](../03-bijux-gnss-infra/index.md)
- [Core handbook](../02-bijux-gnss-core/index.md)
- [Navigation handbook](../04-bijux-gnss-nav/index.md)
- [Signal handbook](../06-bijux-gnss-signal/index.md)

When a workflow proposal starts justifying itself with repository convenience,
inspect the neighbor handbook first anyway. That habit is what keeps
`bijux-gnss-dev` from becoming the place where product behavior or policy debt
gets hidden behind a maintenance label.
