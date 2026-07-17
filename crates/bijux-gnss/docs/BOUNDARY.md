# Boundary

Owner: operator CLI workflows and top-level package façade

## Scope

`bijux-gnss` owns:
- the `bijux` binary target
- command-line parsing and stable command shape
- command orchestration over lower-level crates
- report rendering and operator-oriented output shape
- top-level curated crate re-exports from `lib.rs`

## What this crate must not own

- low-level signal implementations
- navigation estimation internals
- receiver stage internals and runtime engine ownership
- repository run-layout rules and artifact persistence contracts

## Dependency rule

This crate is allowed to depend downward on the product and infrastructure crates because it is the
operator boundary. Those crates should not depend upward on the CLI.

## Effect model

This crate is allowed to own command-facing effects such as reading configs, selecting outputs, and
rendering reports. The lower-level rules for persisted artifacts and runtime execution still belong
to their owning crates.
