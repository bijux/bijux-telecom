---
title: API Surface
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-receiver-docs
last_reviewed: 2026-07-17
---

# API Surface

`bijux-gnss-receiver` publishes one deliberate downstream surface through
`bijux_gnss_receiver::api`.

## Why One Curated Surface Matters

The receiver crate is operationally broad enough that downstream crates
genuinely need runtime types, stage entrypoints, artifacts, and some
lower-owner re-exports. A single curated module keeps those imports organized
by runtime role rather than internal file layout.

## What Belongs In `api.rs`

- exports needed by multiple downstream crates or tools
- runtime records and helpers with durable receiver meaning
- stage, port, or validation seams that callers are expected to implement or
  consume

## What Should Stay Out

- one-caller convenience wrappers
- stage-internal helpers that do not carry a stable receiver contract
- lower-owner science exported only to avoid writing a better receiver boundary

## Closest Proof

- `crates/bijux-gnss-receiver/src/api.rs`
- `crates/bijux-gnss-receiver/API.md`
- `crates/bijux-gnss-receiver/docs/PUBLIC_API.md`
