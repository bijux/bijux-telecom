---
title: Code Navigation
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Code Navigation

Use this route when you need to inspect infra quickly.

## Fast Reading Path

1. start at `src/api.rs` to see the curated infrastructure surface
2. jump to the owning family:
   dataset registry source, raw-IQ metadata source, run-layout source,
   artifact-inspection source, receiver-profile override source, experiment
   source, sweep source, provenance source, or validation-adapter source
3. read the corresponding crate-local docs under
   the infra crate docs, especially the public API, dataset, run-layout,
   override, hashing, validation, and test guides
4. confirm with the narrow infra tests and any relevant root handbook page

## Review Shortcut

If a change touches run layout, dataset registry parsing, or validation
adapters, review it as a repository contract change first and an
implementation-detail change second.

## First Proof Check

Inspect curated public API source, dataset registry source, raw-IQ metadata
source, run-layout source, infra guardrail tests, and override integration
tests. Use the [infra public API guide](../../../crates/bijux-gnss-infra/docs/PUBLIC_API.md)
and [run layout guide](../../../crates/bijux-gnss-infra/docs/RUN_LAYOUT.md) to
keep code navigation tied to reader-facing contracts.
