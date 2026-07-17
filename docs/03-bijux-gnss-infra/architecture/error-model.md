---
title: Error Model
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Error Model

The infra error model should explain repository-state failures clearly without
pretending to own the product errors underneath them.

## What The Model Owns

- invalid dataset registry content
- malformed sidecars or coordinate strings
- run-layout persistence failures
- override and sweep application failures
- artifact inspection and repository-facing validation failures

## What It Does Not Own

- receiver runtime remediation
- navigation-solver refusal logic
- command wording and operator guidance

## Why This Matters

Infrastructure failures should name repository-state problems precisely while
leaving product-science errors to the crates that truly own them.

## First Proof Check

- `crates/bijux-gnss-infra/src/parse/coordinates.rs`
- `crates/bijux-gnss-infra/src/datasets/`
- `crates/bijux-gnss-infra/src/run_layout/`
- `crates/bijux-gnss-infra/src/artifact_inspection/`
- `crates/bijux-gnss-infra/src/validate_reference.rs`
