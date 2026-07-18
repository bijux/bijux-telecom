---
title: Compatibility Commitments
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Compatibility Commitments

This page records what the receiver handbook treats as a durable public
promise.

## Durable Commitments

- `bijux_gnss_receiver::api` remains the main downstream import surface
- public runtime, stage, and port families remain organized by runtime role
  rather than transient file layout
- public artifact and validation types keep their receiver-boundary meaning
  explicit
- re-exported lower-owner surfaces remain identifiable as convenience imports,
  not ownership transfers

## Allowed Internal Freedom

- internal file reshaping behind stable exports
- stage-local helper changes that do not alter public runtime meaning
- deeper decomposition of engine, pipeline, or sim internals when the public
  boundary stays clearer

## Compatibility Review Trigger

If a change alters what a downstream crate can truthfully infer from a public
receiver type, not merely how it is implemented, compatibility review is
required.
