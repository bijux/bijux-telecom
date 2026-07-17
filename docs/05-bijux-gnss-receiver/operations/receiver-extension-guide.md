---
title: Receiver Extension Guide
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Receiver Extension Guide

This page absorbs the old root-level receiver-extension guide into the runtime
owner that actually composes new stages and receiver behavior.

## Extension Sequence

1. implement the runtime behavior in the owning receiver area
2. keep cross-stage records in shared contract types rather than smuggling
   runtime meaning through local hacks
3. emit receiver diagnostics for meaningful stage failures
4. update command wiring only if the public operator surface actually changes

## Boundary Rule

If a capability is reusable signal math or standalone navigation science, it
should move into its owner before the receiver depends on it.
