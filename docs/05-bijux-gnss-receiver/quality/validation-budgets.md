---
title: Validation Budgets
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# Validation Budgets

This page absorbs the older root-level correctness-budget guidance into the
runtime owner that exposes validation and staged execution proof.

## Budget Families

- acquisition error and threshold budgets
- tracking stability and jitter budgets
- runtime-side iteration and gating limits that affect receiver validation

## Reader Rule

The exact numbers still need proof in tests and validation workflows. What this
page anchors is the receiver-side ownership of those validation budgets once
they become part of top-level runtime trust.

## First Proof Check

- `crates/bijux-gnss-receiver/docs/REFERENCE_VALIDATION.md`
- `crates/bijux-gnss-receiver/tests/integration_navigation_pvt_accuracy_budget.rs`
- `crates/bijux-gnss-receiver/tests/integration_tracking_accuracy_budget.rs`
