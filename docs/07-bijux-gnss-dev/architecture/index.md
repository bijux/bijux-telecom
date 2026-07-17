---
title: Architecture
audience: mixed
type: index
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Architecture

Open this section when the question is how `bijux-gnss-dev` organizes its
binary command surface and repository-scoped effects in code.

## What This Section Covers

- the command layout inside `src/main.rs`
- dependency direction and effect boundaries
- benchmark comparison flow and governed-file validation structure
- the deliberate absence of a reusable library surface

## First Code Roots

- `crates/bijux-gnss-dev/src/main.rs`
- `crates/bijux-gnss-dev/tests/integration_guardrails.rs`
- `crates/bijux-gnss-dev/tests/integration_nextest_suite_selection.rs`
