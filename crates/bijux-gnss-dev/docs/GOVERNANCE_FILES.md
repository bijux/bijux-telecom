# Governance Files

`bijux-gnss-dev` exists to keep a few repository control files governed rather than folkloric.

## Owned files

- `audit-allowlist.toml`
- `configs/rust/deny.deviations.toml`
- `benchmarks/bencher_baseline.txt`

## Ownership model

This crate does not own every repository config file. It owns the quality gates for the files above
because they represent security exceptions, standards deviations, and benchmark evidence that can
quietly rot without an explicit maintainer boundary.

## Documentation obligation

When a new repository-governance file becomes important enough to require typed validation or
derived command behavior, it should be added here and documented as an owned surface rather than
being smuggled into `src/main.rs` as one more path constant.
