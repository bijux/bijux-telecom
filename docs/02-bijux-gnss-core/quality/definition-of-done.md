---
title: Definition of Done
audience: mixed
type: quality
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Definition of Done

A `bijux-gnss-core` change is not done when code compiles. It is done when the
contract change, if any, is legible and defended.

## Completion Gate

| changed surface | done means | first proof |
| --- | --- | --- |
| public exports | `api.rs` exposes only deliberate shared contracts | `cargo test -p bijux-gnss-core --test public_api_guardrail` |
| artifact payloads | validation rejects incoherent payloads and preserves versioned meaning | `nav_artifact_validation` or `tracking_artifact_validation` |
| observation and tracking records | receiver and nav can read the same record meaning without solver or runtime leakage | contract docs plus the changed record tests |
| time, units, and coordinates | edge cases remain explicit and portable across consumers | `cargo test -p bijux-gnss-core --test prop_timekeeping` |
| diagnostics and support matrix | shared diagnostic and capability records remain stable across runtime, infra, and command layers | support/diagnostic docs plus guardrail proof |
| serialization | old and new readers can still interpret persisted meaning honestly | `SERIALIZATION.md` plus artifact validation proof |

## Not Done Yet Means

- a helper was made public before it had shared-contract meaning
- serialized meaning changed but the docs still describe the old behavior
- invariants are implicit in test names only
- the crate absorbed behavior because it was easier than naming a downstream
  owner

## Reader Questions Before Commit

- Which contract family moved?
- Which higher crates read that meaning today?
- Is the change versioned, serialized, or public?
- What fixture or property test would catch a wrong interpretation?
- Did any product behavior move into core because it was convenient?

## Proof Route

Read `crates/bijux-gnss-core/docs/CONTRACTS.md`,
`CONTRACT_MAP.md`, `PUBLIC_API.md`, `SERIALIZATION.md`, and `TESTS.md` as
needed for the changed family. A core change is done only when the public
promise, contract prose, source behavior, and protecting tests all describe the
same shared meaning.
