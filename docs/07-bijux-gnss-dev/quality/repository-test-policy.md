---
title: Repository Test Policy
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Repository Test Policy

This page replaces the old root-level testing guide. It records the proof
policy maintainers are expected to preserve across fast, slow, and frozen
lanes.

## Test Families

- unit tests stay close to implementation when locality improves clarity
- integration, golden, and fault tests live where they prove crate boundaries
- property tests must keep bounded ranges and durable regression seeds
- roster-governance tests must prove that named slow tests still resolve to
  real repository test functions

## Lane Discipline

- `make test` is the fast lane and must exclude governed slow tests
- `make test-all` and `make test-all-frozen` are the complete proof lanes
- frozen lanes are for pinned-snapshot verification rather than mutable
  worktree confidence

## Time Budget Rule

Tests that routinely exceed the fast-lane budget must be classified and routed
out of `make test`. Tests that cannot justify their runtime even in the full
lane should be split, reduced, or removed.

## Maintainer Rule

If a change touches `configs/rust/nextest-slow-roster.txt`, run
`cargo test -p bijux-gnss-dev --test integration_nextest_suite_selection`
because that file is defended by test policy rather than by a maintainer
command.
