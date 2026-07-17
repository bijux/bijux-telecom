---
title: Review Scope
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-signal-docs
last_reviewed: 2026-07-17
---

# Review Scope

Reviewers should align their depth to the changed owner.

## Scope By Change Type

- catalog change:
  inspect signal meaning, registry entries, and downstream physical
  interpretation
- code-family change:
  inspect canonical constants, assignments, samplers, and reference proof
- DSP change:
  inspect mathematical role, runtime neutrality, and long-duration behavior
- raw-IQ or sample change:
  inspect contract meaning, quantization assumptions, and downstream
  compatibility
- validation change:
  inspect whether the rule is still signal-layer judgment rather than a higher
  policy decision

## Review Shortcut

If the diff touches `src/api.rs`, treat the change as public-boundary review
even when the implementation edit looks small.
