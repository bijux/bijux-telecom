---
title: Facade Contracts
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# Facade Contracts

The Rust facade is intentionally smaller than the binary surface.

## Owned Facade Surface

- re-exports of `core`, `receiver`, `signal`, and feature-gated `nav`

## What The Facade Is For

- package-level convenience when a consumer genuinely wants one crate import
- making the main GNSS stack discoverable without treating the CLI crate as a
  bespoke helper library

## What The Facade Is Not For

- hosting command-only helpers
- masking lower-owner boundaries
- growing a parallel public API unrelated to the binary surface

## Closest Proof

- `crates/bijux-gnss/src/lib.rs`
- `crates/bijux-gnss/docs/FACADE.md`
- `crates/bijux-gnss/docs/PUBLIC_API.md`
