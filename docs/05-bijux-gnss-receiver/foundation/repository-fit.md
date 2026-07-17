---
title: Repository Fit
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Repository Fit

`bijux-gnss-receiver` is the runtime center of the repository. It is not the
command layer and not the persistence layer.

## Why The Repository Needs This Layer

- runtime execution should be reusable across commands, synthetic validation,
  and downstream artifact workflows
- lower scientific owners should not be forced to absorb scheduling, sinks, and
  runtime state just because they are used during execution
- higher layers should consume a receiver boundary instead of rebuilding one
  ad hoc

## How It Fits With Neighboring Handbooks

- [04-bijux-gnss-nav](../../04-bijux-gnss-nav/) explains the navigation
  science this runtime may call but should not redefine
- [03-bijux-gnss-infra](../../03-bijux-gnss-infra/) explains persisted
  repository ownership that may consume receiver artifacts but should not own
  runtime composition
- [01-bijux-gnss](../../01-bijux-gnss/) explains command ownership above this
  runtime surface

## The Fit To Defend

This crate should remain the place where a receiver run is explainable without
opening command code, repository persistence code, or lower-level scientific
owners to recover runtime intent.
