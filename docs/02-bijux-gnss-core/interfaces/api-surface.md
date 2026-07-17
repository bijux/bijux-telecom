---
title: API Surface
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# API Surface

`bijux-gnss-core` exports one deliberate downstream surface:
`bijux_gnss_core::api`.

## Public Module Policy

- `src/lib.rs` exposes `pub mod api;`
- implementation modules stay private unless they are intentionally re-exported
- new public structs and free functions should be reviewed as contract changes,
  not as convenience changes

## Main Public Families

- artifact contracts for versioned acquisition, tracking, observation,
  navigation, and support-matrix envelopes
- configuration and diagnostics contracts for schema versions, validation
  reports, diagnostic codes, event summaries, and canonical error families
- foundational physical types for identity, time systems, units, and geodetic
  coordinates
- observation and navigation records for acquisition, tracking, differencing,
  support inventory, and solution-state exchange

## Why The Policy Exists

Without one curated public surface, higher-level crates would start depending
on internal module paths. That would make refactors artificially expensive and
weaken the distinction between stable meaning and internal layout.

## Protecting Proof

- `crates/bijux-gnss-core/docs/PUBLIC_API.md`
- `crates/bijux-gnss-core/docs/CONTRACTS.md`
- `crates/bijux-gnss-core/docs/INVARIANTS.md`
- `crates/bijux-gnss-core/docs/CONTRACT_MAP.md`
- `crates/bijux-gnss-core/tests/public_api_guardrail.rs`
