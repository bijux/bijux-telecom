# Façade

`bijux-gnss` exposes a narrow package-level Rust façade in addition to its operator CLI.

## Façade responsibilities

`src/lib.rs` exists so downstream Rust users can reach the main GNSS stack through one package
surface instead of importing several lower-level crates directly for the simplest use cases.

## Boundary rule

This façade should stay narrow:

- re-export owned lower-level crate surfaces deliberately
- avoid adding bespoke helper functions that blur ownership
- avoid turning the CLI crate into a second mixed-responsibility library

If a capability is really owned by `core`, `signal`, `receiver`, `nav`, or `infra`, improve the
owning crate instead of growing this façade.
