---
title: Common Workflows
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Common Workflows

This page describes the recurring edit patterns in `bijux-gnss-receiver`.

## Add Or Change Runtime Configuration

- confirm the change belongs in runtime composition rather than command policy
- update receiver-facing docs if the meaning of a configuration field moves
- run the smallest execution tests that prove the changed behavior
- inspect whether the change affects `api.rs` or only internal runtime logic

## Change One Stage Family

- identify whether the change is local to acquisition, tracking, observations,
  or receiver-owned navigation adapters
- run the most local stage tests first
- then run the narrowest integration tests that prove the runtime-visible
  effect

## Change Ports Or Runtime Sinks

- confirm the change is execution-oriented, not repository-oriented
- check both the public port contract and the concrete adapters
- run tests that exercise the affected source, sink, or timing seam

## Change Validation Or Synthetic Proof

- explain whether the runtime behavior changed or the proof expectation changed
- run the relevant validation-report or synthetic test families
- avoid broadening tolerances just to make a runtime proof green

## First Proof Check

Read `crates/bijux-gnss-receiver/docs/RUNTIME.md`,
`crates/bijux-gnss-receiver/docs/PIPELINE.md`,
`crates/bijux-gnss-receiver/docs/PORTS.md`,
`crates/bijux-gnss-receiver/docs/ARTIFACTS.md`,
`crates/bijux-gnss-receiver/docs/REFERENCE_VALIDATION.md`,
`crates/bijux-gnss-receiver/docs/SIMULATION.md`, and
`crates/bijux-gnss-receiver/docs/TESTS.md` before editing. Those surfaces map
a proposed change to the right runtime owner and proof family.
