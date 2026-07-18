---
title: Compatibility Commitments
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# Compatibility Commitments

This page records what the nav handbook treats as a durable public promise.

## Durable Commitments

- `bijux_gnss_nav::api` remains the main downstream import surface
- public constellation and product families remain organized by scientific role
  rather than by transient file layout
- public refusal, rejection, and evidence types keep their scientific meaning
  explicit
- runtime-neutral solver and provider surfaces stay consumable without
  repository-layout knowledge

## Allowed Internal Freedom

- internal file reshaping behind stable exports
- solver-local helper changes that do not alter public scientific meaning
- deeper decomposition of parser or estimator internals when exports remain
  stable and clearer

## Compatibility Review Trigger

If a change alters what a downstream crate can truthfully infer from a public
type, not merely how it is implemented, compatibility review is required.
