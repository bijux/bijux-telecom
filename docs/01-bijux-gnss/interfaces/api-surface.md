---
title: API Surface
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-docs
last_reviewed: 2026-07-17
---

# API Surface

`bijux-gnss` has two public surfaces: the `bijux` binary and the thin Rust
facade from `src/lib.rs`.

## Why Two Surfaces Matter

The binary is the primary operator contract. The Rust facade exists for
package-level convenience, but it must stay visibly secondary so the command
crate does not become a second mixed-responsibility API layer.

## What Belongs In The Public Surface

- stable command families, flags, and report shape
- top-level workflow composition that operators genuinely interact with
- a small, durable facade over lower-level crates when that convenience is
  justified

## What Should Stay Out

- one-off command helpers with no durable operator meaning
- lower-owner science exported through the facade only to save imports
- repository or runtime internals that are not actually command-boundary
  promises

## Closest Proof

- `crates/bijux-gnss/src/main.rs`
- `crates/bijux-gnss/src/lib.rs`
- `crates/bijux-gnss/API.md`
