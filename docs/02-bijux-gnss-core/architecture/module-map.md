---
title: Module Map
audience: mixed
type: explanation
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Module Map

The module map matters because `bijux-gnss-core` is intentionally dense. The
crate works only when each dense area still has a durable contract family to
own.

## Main Ownership Map

| code area | primary ownership |
| --- | --- |
| `src/api.rs` | curated downstream surface |
| `src/artifact/` | versioned artifact envelopes, payload versions, and validation traits |
| `src/config.rs` | schema versioning and validation-report shape |
| `src/diagnostic/` and `src/error.rs` | diagnostic codes, severity, and canonical error families |
| `src/ids.rs` | constellation, satellite, signal, and registry identities |
| `src/time.rs` | GPS, UTC, TAI, receiver-sample time, and leap-second meaning |
| `src/units.rs` and `src/geo.rs` | strong physical units and geodetic coordinate types |
| `src/observation/` and `src/observation_quality.rs` | acquisition, tracking, observation, differencing, and quality records |
| `src/nav_solution.rs` | solution epochs, residuals, inter-system bias, and lifecycle-state records |
| `src/support_matrix.rs` | support inventory records shared across runtime, infra, and CLI |

## Why This Map Exists

The map is less about memorizing files and more about preventing weak
"shared"-type placement. A type that belongs in one of these families should
usually deepen that family rather than create a new vague top-level concept.

## First Proof Check

- `crates/bijux-gnss-core/docs/CONTRACT_MAP.md`
- `crates/bijux-gnss-core/src/lib.rs`
