# Public API

`bijux-gnss` has two different public surfaces.

## Binary surface

The primary public surface is the `bijux` command-line binary. That is where operator workflows are
owned and versioned.

## Rust surface

`src/lib.rs` exposes a small façade of lower-level crates:
- `core`
- `receiver`
- `signal`
- `nav` when the `nav` feature is enabled

This façade is intentionally small. It exists so downstream Rust users can reach the main GNSS
stack through one package without treating the CLI crate as a place for bespoke helpers.

## Extension rule

If a new Rust export is really owned by `core`, `signal`, `receiver`, `nav`, or `infra`, keep it in
the owning crate instead of growing `bijux-gnss` into a second mixed-responsibility API layer.
