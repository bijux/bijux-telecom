---
title: Change Sequence
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Change Sequence

Use this sequence for any substantive `bijux-gnss-infra` change.

1. decide whether the work truly belongs in infra
2. identify the repository contract family being changed
3. update the owning crate-local docs if the repository meaning changes
4. update the root handbook pages here if package-level reader guidance changes
5. run the narrowest protecting tests
6. only then inspect higher-level command or runtime fallout

Skipping the first step is the most expensive mistake. A stronger owner is
often a cleaner fix than a permanent new infra contract.
