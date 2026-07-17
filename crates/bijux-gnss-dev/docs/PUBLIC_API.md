# Public API

`bijux-gnss-dev` does not publish a Rust library API.

## Public surface

- the durable public surface is the `bijux-gnss-dev` binary command set
- `src/main.rs` is intentionally binary-owned
- there is no `lib.rs`, and downstream crates should not depend on this crate as a library

## Why that is intentional

This crate owns repository maintenance workflows. A Rust library surface would encourage other
crates to couple themselves to maintainer-only internals instead of keeping repository governance at
the edge.

## Stability rule

If a capability here needs reuse by product crates, the reusable part should move into the owning
product or policy crate and this binary should call into that stable surface. `bijux-gnss-dev`
itself should remain a binary boundary.
