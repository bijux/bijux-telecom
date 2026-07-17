---
title: Durable Naming
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Durable Naming

This page replaces the old root-level naming guide. It records the maintainer
language rules that keep the repository readable long after one delivery cycle
is forgotten.

## Preferred Public Terms

- observations rather than `obs`
- ephemeris rather than `eph` in public surfaces
- products when the scope is broad rather than overloading a narrow internal
  type name
- constellation, signal band, carrier phase, and pseudorange when the reader
  needs domain-accurate language

## Names To Reject

- numbered or sequence-driven file names such as `foo_1.rs` or `new2.rs`
- generic buckets such as `helpers.rs`, `support.rs`, or `misc.rs`
- public names that abbreviate away domain meaning when the longer form is the
  real contract

## Reader Rule

If a name only tells the story of when a thing was introduced, renamed, or
temporarily parked, it is not durable enough for this repository.
