---
title: Numerical Budgets
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Numerical Budgets

This page keeps the old root-level numerical-budget guidance close to the
shared contract owner rather than leaving it as unaffiliated root prose.

## Budget Families

- acquisition tolerance language that downstream validation consumes
- tracking stability expectations that multiple crates may reference
- navigation residual and convergence budget language when it becomes a shared
  contract rather than one local implementation detail

## Reader Rule

Budget numbers still need proof in the owning runtime or navigation crate. What
belongs here is the shared contract language around those tolerances and why
downstream readers should interpret them consistently.
