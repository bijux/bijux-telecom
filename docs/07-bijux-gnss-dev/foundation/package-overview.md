---
title: Package Overview
audience: mixed
type: foundation
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Package Overview

`bijux-gnss-dev` is the repository-maintenance binary for `bijux-telecom`. It
centralizes a small number of reviewed workflows that are important enough to
deserve typed validation and explicit ownership.

The crate has four durable centers of gravity:

- audit-exception discipline for `audit-allowlist.toml`
- deny-policy deviation discipline for `configs/rust/deny.deviations.toml`
- derived audit-ignore arguments from one reviewed exception source
- benchmark comparison and evidence emission for a curated maintainer benchmark
  set

The crate is intentionally a binary-only boundary. It is meant to sit at the
repository edge where governance inputs, diagnostics, and benchmark evidence
need an owner without turning maintainer behavior into a reusable product API.
