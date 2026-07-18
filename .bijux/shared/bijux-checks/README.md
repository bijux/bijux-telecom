# Bijux Shared Standards

This directory owns shared-directory synchronization, checksum validation, and
standards check execution.

## Capability Selection

Repositories may select managed shared libraries with
`BIJUX_STD_CAPABILITIES`. The `common` capability is always included.

```bash
BIJUX_STD_CAPABILITIES="docs rust" make bijux-std-update
BIJUX_STD_CAPABILITIES="docs rust" make bijux-std-checks
```

Available capabilities are declared in `bijux-std-checks.yml`:

- `common`: shared checks, GitHub governance, and language-neutral Make modules.
- `docs`: shared documentation shell and assets.
- `python`: Python Make modules.
- `rust`: Rust Make modules.

When capabilities are explicitly selected, the updater removes unselected
managed directories and writes a manifest containing only selected directories.
Without `BIJUX_STD_CAPABILITIES`, synchronization retains the complete shared
set for existing consumers.

## Source Pinning

`BIJUX_STD_REF` accepts an exact commit SHA, branch, or tag. Downstream
repositories should use an accepted exact commit SHA. The updater fetches and
checks out that object in detached mode before copying managed content.

## Artifact Placement

Checkout, staging, rendered validation input, and Python cache output belong
under the consumer repository's `artifacts/` directory.
