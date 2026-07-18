---
title: Known Limitations
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Known Limitations

The receiver crate is the workspace's execution center, but it is not the
source of every GNSS truth. These limits are reader-facing guardrails: they
explain what current receiver proof can and cannot justify.

## Limits Readers Should Know

| limitation | practical consequence | how to read the proof honestly |
| --- | --- | --- |
| Broad public API | Downstream crates consume runtime behavior directly, so export changes can become compatibility changes even when the code diff is small. | Check `crates/bijux-gnss-receiver/docs/PUBLIC_API.md` before treating an export as internal. |
| Expensive integration surface | Full receiver proof is costly; local checks must be chosen by contract, not convenience. | Start from `crates/bijux-gnss-receiver/docs/TESTS.md` and name the missing proof when a full lane is not run. |
| Stage coupling | Acquisition, tracking, observations, and navigation handoff interact under real signal conditions. | A stage-local test is necessary but may not be sufficient when state crosses a stage boundary. |
| Synthetic truth | Synthetic captures are excellent for bounded regression proof, but they do not replace independent field evidence. | Read synthetic tests as receiver-boundary proof, not as a claim that every physical environment is covered. |
| Artifact interpretation | Receiver artifacts describe runtime output before infra decides repository layout. | Do not infer persisted path, registry, or history semantics from receiver artifact structs. |

## What This Handbook Does Not Claim

- It does not claim every constellation, signal band, interference profile, or
  oscillator behavior is validated by the same test family.
- It does not claim navigation science is receiver-owned merely because the
  receiver can call navigation helpers.
- It does not claim repository artifact persistence is receiver-owned merely
  because receiver output is later written to disk.
- It does not claim broad workspace success when only a narrow stage proof was
  run.

## First Proof Route

Read these in order when evaluating a receiver claim:

1. `crates/bijux-gnss-receiver/docs/BOUNDARY.md`
2. `crates/bijux-gnss-receiver/docs/TESTS.md`
3. `crates/bijux-gnss-receiver/tests/prop_receiver.rs`
4. `crates/bijux-gnss-receiver/tests/integration_pipeline_determinism.rs`
5. the stage-specific integration test named by the changed behavior

The limitation is acceptable only when the documentation names it plainly and
the selected proof matches the claim being made.
