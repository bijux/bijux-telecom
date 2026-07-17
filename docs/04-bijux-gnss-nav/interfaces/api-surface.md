---
title: API Surface
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-nav-docs
last_reviewed: 2026-07-17
---

# API Surface

`bijux-gnss-nav` publishes one deliberate downstream surface through
`bijux_gnss_nav::api`.

## Why One Curated Surface Matters

The crate is scientifically broad enough that downstream packages genuinely
need many families. A single curated module keeps those imports organized by
scientific role rather than by internal file layout.

## What Belongs In `api.rs`

- exports needed by multiple downstream crates or tools
- typed records and helpers with durable scientific meaning
- provider or estimator seams that callers are expected to implement or consume

## What Should Stay Out

- parser-local helpers
- solver-internal state manipulation that matters only inside one family
- convenience exports added for one short-lived caller

## Closest Proof

- `crates/bijux-gnss-nav/src/api.rs`
- `crates/bijux-gnss-nav/API.md`
- `crates/bijux-gnss-nav/docs/PUBLIC_API.md`
