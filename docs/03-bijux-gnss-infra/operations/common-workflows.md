---
title: Common Workflows
audience: mixed
type: explanation
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Common Workflows

Most infra changes fall into one of five workflow families.

## Dataset Interpretation Change

You are changing registry parsing, sidecar lookup, coordinate parsing, or
capture provenance handling. Treat it as shared repository behavior, not one
command fix.

## Run Footprint Change

You are changing run identity, manifest shape, report shape, history entries,
or artifact header behavior. This is a durability-sensitive change.

## Override Or Sweep Change

You are changing how maintained configs become run variants. Review typed
variation behavior first, not just caller ergonomics.

## Artifact Inspection Change

You are changing how persisted artifacts are explained or validated after
execution. Keep the distinction from runtime validation explicit.

## Provenance Change

You are changing hash capture or reproducibility evidence. Review whether the
change affects how future runs are compared or audited.
