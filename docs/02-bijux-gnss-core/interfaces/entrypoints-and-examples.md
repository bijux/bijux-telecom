---
title: Entrypoints and Examples
audience: mixed
type: explanation
status: canonical
owner: bijux-gnss-core-docs
last_reviewed: 2026-07-17
---

# Entrypoints and Examples

The main entrypoint is the crate import surface itself.

## Example: Foundational Types

```rust
use bijux_gnss_core::api::{Constellation, GpsTime, SatId};

let sat = SatId { constellation: Constellation::Gps, prn: 7 };
let t = GpsTime { week: 2200, tow_s: 1000.0 };
```

## Example: Artifact Validation

```rust
use bijux_gnss_core::api::{ArtifactValidate, ArtifactV1};

fn validate(artifact: &ArtifactV1) -> bool {
    artifact.validate().is_ok()
}
```

## Example: Observation Contract Use

```rust
use bijux_gnss_core::api::ObsEpoch;

fn accept_epoch(_epoch: &ObsEpoch) {
    // Downstream crates consume the shared record shape without importing
    // private module paths.
}
```

The examples are intentionally small. This crate is a contract surface, so the
best examples are import and exchange examples, not full workflow programs.
