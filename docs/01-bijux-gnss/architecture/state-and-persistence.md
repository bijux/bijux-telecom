---
title: State And Persistence
audience: mixed
type: architecture
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# State And Persistence

`bijux-gnss` owns command-boundary state, but very little durable product state.

## State It Owns

- parsed command and argument state
- command-runtime environment and output-routing state
- operator-facing report assembly state
- thin facade exports and command-level workflow selection state

## Persistence It Does Not Own

- repository run layout, manifests, and indexed artifact history
- lower-level runtime artifact content beyond what is rendered or forwarded
- scientific truth records owned by receiver, nav, signal, or infra

## Why The Distinction Matters

Commands may read configs, bundle outputs, or render reports, but that does not
make persistence policy or scientific state command-owned.

## Closest Proof

- `crates/bijux-gnss/src/cli/command_line.rs`
- `crates/bijux-gnss/src/cli/command_runtime/`
- `crates/bijux-gnss/src/cli/report.rs`
