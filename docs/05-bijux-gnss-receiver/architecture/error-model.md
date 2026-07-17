---
title: Error Model
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Error Model

Receiver runtime has more than one honest failure mode, and the architecture
should keep them distinct.

## Main Failure Families

- source or sample errors at the runtime boundary
- configuration and schema validation failures before execution begins
- stage-local refusal or insufficient evidence during acquisition, tracking, or
  observations
- feature-gated unavailability when navigation-dependent behavior is not
  enabled
- validation reports or comparison results that signal degraded trust rather
  than a malformed runtime

## Why Distinction Matters

A sample-source failure is not the same as a weak acquisition candidate, and
neither is the same as a receiver-boundary validation report that shows the run
is scientifically poor. If those cases collapse into generic failure handling,
review becomes much weaker.

## Closest Proof

- `crates/bijux-gnss-receiver/src/engine/receiver_config.rs`
- `crates/bijux-gnss-receiver/src/io/data.rs`
- `crates/bijux-gnss-receiver/src/reference_validation.rs`
- `crates/bijux-gnss-receiver/src/validation_report.rs`
