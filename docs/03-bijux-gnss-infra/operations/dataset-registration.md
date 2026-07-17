---
title: Dataset Registration
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Dataset Registration

This page absorbs the old root-level dataset guide into the repository owner
that actually controls dataset registration and sidecar interpretation.

## Registration Sequence

1. place the capture data and reviewed sidecar material under the repository
   dataset area
2. add the dataset registry entry with stable dataset identity and capture
   metadata
3. include sidecar fields when extra ingest metadata such as offsets or notes
   are part of the reviewed contract
4. run the relevant infra validation path before relying on the dataset in
   higher-level workflows

## Boundary Rule

The infrastructure crate owns dataset registration mechanics. Higher-level
crates may consume dataset identity, but they should not redefine how the
repository names and validates captures.
