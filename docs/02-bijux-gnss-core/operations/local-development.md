---
title: Local Development
audience: mixed
type: operations
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Local Development

Local work in `bijux-gnss-core` should start from contract meaning, not from the
downstream crate that first exposed the need. This crate is the vocabulary layer
for artifacts, observations, navigation solutions, identities, diagnostics,
time, units, and support matrices.

## Productive Local Loop

1. Read `crates/bijux-gnss-core/docs/CONTRACTS.md` and `CONTRACT_MAP.md`.
2. Inspect `PUBLIC_API.md` before adding or moving an export.
3. Check `SERIALIZATION.md` before changing any persisted or versioned record.
4. Make the smallest coherent shared-contract change.
5. Run the focused proof from `TESTS.md`.
6. Expand to downstream crates only when their interpretation genuinely moves.

## Local Navigation

| question | open first |
| --- | --- |
| "Is this a shared record or a receiver/nav helper?" | `crates/bijux-gnss-core/docs/BOUNDARY.md` |
| "Which module owns this contract?" | `crates/bijux-gnss-core/docs/CONTRACT_MAP.md` |
| "Can callers import this deliberately?" | `crates/bijux-gnss-core/docs/PUBLIC_API.md` |
| "Will old artifacts still mean the same thing?" | `crates/bijux-gnss-core/docs/SERIALIZATION.md` |
| "Which tests protect the claim?" | `crates/bijux-gnss-core/docs/TESTS.md` |

## Why Narrow First

Broad workspace runs can tell you that something broke, but they do not tell
you whether the shared contract is correct. Core development is trustworthy
when the narrow test proves the shared meaning first and downstream checks only
confirm that consumers still agree.

Do not prototype a core type only because it is convenient. If the type cannot
be described as a stable shared meaning in one of the contract families, it
belongs in a higher owner.
