# Naming

Canonical terms
- observations (not obs)
- ephemeris (not eph in public APIs)
- products (not precise_products unless scoped)
- constellation
- signal band
- carrier phase
- pseudorange

Banned aliases
- `obs` in filenames and public API types
- `eph` in filenames and public API types
- `ppp_impl_*` / `*_1.rs` / `*_2.rs`

Public API surface
- Keep crate exports minimal: modules + key types only.
- Internal modules stay private.


## Banned File Names

Avoid generic container file names. The following are banned:

- `helpers.rs`
- `support.rs`
- `misc.rs`

Use concept-specific names instead (`paths.rs`, `format.rs`, `units.rs`, etc.).
