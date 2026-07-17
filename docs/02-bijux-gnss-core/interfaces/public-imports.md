---
title: Public Imports
audience: mixed
type: interfaces
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Public Imports

Downstream crates should import shared core meaning through
`bijux_gnss_core::api`, not through private module paths.

## Main Import Families

- artifact and validation families
- configuration and diagnostic families
- foundational identity, time, unit, and geometry families
- observation and tracking record families
- navigation-solution and support-matrix families

## Import Rule

If a downstream crate needs a type that exists only in a private module path,
that is a signal to review whether the type should be re-exported or whether
the downstream crate is reaching too far inward.

## Good Import Shape

```rust
use bijux_gnss_core::api::{GpsTime, ObsEpoch, SatId, ValidationReport};
```

## Bad Import Shape

```rust
use bijux_gnss_core::observation::epochs::ObsEpoch;
```

The second style couples callers to layout instead of to the stable public
surface.

## Protecting Proof

Inspect `crates/bijux-gnss-core/src/api.rs`,
`crates/bijux-gnss-core/docs/PUBLIC_API.md`, and
`crates/bijux-gnss-core/tests/public_api_guardrail.rs` to confirm the import
families documented here still match the curated public surface.
