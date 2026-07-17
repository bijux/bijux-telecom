---
title: Public Imports
audience: mixed
type: explanation
status: canonical
owner: bijux-gnss-infra-docs
last_reviewed: 2026-07-17
---

# Public Imports

Callers should import repository-facing helpers through `bijux_gnss_infra::api`
rather than reaching into private module paths.

## Main Import Families

- dataset registry and raw-IQ metadata helpers
- run directory layout, manifest, report, and history helpers
- override and sweep helpers
- artifact explanation and validation entrypoints
- provenance hashing helpers
- validation-reference helpers and selected lower-level convenience re-exports

## Import Rule

If a caller needs a private infra module path directly, that is either a sign
that the public surface is incomplete or that the caller is reaching past the
repository contract.

## Good Import Shape

```rust
use bijux_gnss_infra::api::{DatasetRegistry, RunManifest, expand_sweep};
```

## Bad Import Shape

```rust
use bijux_gnss_infra::run_layout::records::RunManifest;
```

The second style couples callers to file layout instead of to the stable
infrastructure surface.
