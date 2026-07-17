---
title: Performance And Profiling
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Performance And Profiling

This page absorbs the older root-level performance playbook into the runtime
owner that carries the hottest staged execution paths.

## Reader Rule

Profile and benchmark the receiver when the question is staged acquisition,
tracking, observation building, or runtime-side validation cost.

## Practical Focus

- bench the runtime path that changed instead of rerunning unrelated crates by
  habit
- profile stage transitions and hot loops where execution ownership lives
- treat benchmark evidence as maintainer-facing proof once the workflow hands
  off to `bijux-gnss-dev`
