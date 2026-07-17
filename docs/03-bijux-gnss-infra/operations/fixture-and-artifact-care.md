---
title: Fixture and Artifact Care
audience: mixed
type: explanation
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Fixture and Artifact Care

Infra is the crate most likely to change how repository artifacts are arranged
or interpreted. That makes artifact care part of ordinary change discipline.

## Care Rules

- treat manifests, reports, and history entries as durable evidence
- change run-footprint meaning only with explicit documentation updates
- if artifact interpretation changes, explain whether the payload meaning or
  only the repository-facing reading changed
- avoid casual churn in examples or checked-in footprint expectations

## Why This Matters

Repository artifacts are often read long after the command and process that
created them are gone. A sloppy infra change can make old evidence harder to
trust without any compiler error.
