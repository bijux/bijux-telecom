---
title: Binary Boundary
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-dev-docs
last_reviewed: 2026-07-17
---

# Binary Boundary

`bijux-gnss-dev` intentionally does not publish a Rust library API.

## What Callers May Rely On

- the binary name `bijux-gnss-dev`
- the documented command inventory
- the governed inputs and outputs named by the maintainer workflows

## What Callers Must Not Assume

- that internal helper functions are a reusable API
- that the crate will expose `lib.rs`
- that product crates should depend on maintainer internals instead of their
  own owning surfaces
